#include "k210_protocol.h"
#include "bsp_uart.h"
#include "bsp_timer.h"
#include <string.h>

/* ==================== 内部变量 ==================== */
static k210_data_t k210_data;

/* 解析状态机 */
typedef enum
{
    K210_WAIT_HEAD1 = 0,
    K210_WAIT_HEAD2,
    K210_WAIT_TYPE,
    K210_WAIT_LEN,
    K210_WAIT_PAYLOAD,
    K210_WAIT_CHECKSUM
} k210_parse_state_t;

static k210_parse_state_t k210_state = K210_WAIT_HEAD1;

static uint8_t  k210_frame_type = 0;
static uint8_t  k210_frame_len = 0;
static uint8_t  k210_frame_idx = 0;
static uint8_t  k210_checksum = 0;
static uint8_t  k210_payload[K210_FRAME_MAX_PAYLOAD];

/* ==================== 内部函数声明 ==================== */
static void K210_ResetParser(void);
static void K210_ProcessFrame(uint8_t type, uint8_t len, const uint8_t *payload);
static int16_t K210_ReadInt16LE(const uint8_t *p);
static void K210_UpdateTimeout(uint32_t now);

/* ==================== 对外接口实现 ==================== */
void K210_Init(void)
{
    K210_ClearData();
    K210_ResetParser();
}

void K210_ClearData(void)
{
    memset(&k210_data, 0, sizeof(k210_data));
    k210_data.line_valid = 0;
    k210_data.qr_valid = 0;
}

const k210_data_t *K210_GetData(void)
{
    return &k210_data;
}

int16_t K210_GetLineOffset(void)
{
    return k210_data.line_offset;
}

int16_t K210_GetQrX(void)
{
    return k210_data.qr_x;
}

int16_t K210_GetQrY(void)
{
    return k210_data.qr_y;
}

uint8_t K210_LineAvailable(void)
{
    return k210_data.line_valid;
}

uint8_t K210_QRAvailable(void)
{
    return k210_data.qr_valid;
}

/*
 * 任务函数：周期调用
 * 1. 从 UART2 环形队列读取所有可用字节
 * 2. 按协议状态机解析
 * 3. 更新超时状态
 */
void K210_ParseTask(void)
{
    uint8_t ch;
    uint32_t now;

    while (UART2_GetChar(&ch))
    {
        switch (k210_state)
        {
            case K210_WAIT_HEAD1:
            {
                if (ch == K210_FRAME_HEAD1)
                {
                    k210_state = K210_WAIT_HEAD2;
                }
                break;
            }

            case K210_WAIT_HEAD2:
            {
                if (ch == K210_FRAME_HEAD2)
                {
                    k210_state = K210_WAIT_TYPE;
                }
                else if (ch == K210_FRAME_HEAD1)
                {
                    /* 仍然可能是新的帧头第1字节 */
                    k210_state = K210_WAIT_HEAD2;
                }
                else
                {
                    k210_state = K210_WAIT_HEAD1;
                }
                break;
            }

            case K210_WAIT_TYPE:
            {
                k210_frame_type = ch;
                k210_checksum = ch;
                k210_state = K210_WAIT_LEN;
                break;
            }

            case K210_WAIT_LEN:
            {
                k210_frame_len = ch;
                k210_checksum = (uint8_t)(k210_checksum + ch);
                k210_frame_idx = 0;

                if (k210_frame_len > K210_FRAME_MAX_PAYLOAD)
                {
                    /* 长度非法，丢帧 */
                    K210_ResetParser();
                }
                else if (k210_frame_len == 0)
                {
                    k210_state = K210_WAIT_CHECKSUM;
                }
                else
                {
                    k210_state = K210_WAIT_PAYLOAD;
                }
                break;
            }

            case K210_WAIT_PAYLOAD:
            {
                k210_payload[k210_frame_idx++] = ch;
                k210_checksum = (uint8_t)(k210_checksum + ch);

                if (k210_frame_idx >= k210_frame_len)
                {
                    k210_state = K210_WAIT_CHECKSUM;
                }
                break;
            }

            case K210_WAIT_CHECKSUM:
            {
                if (ch == k210_checksum)
                {
                    K210_ProcessFrame(k210_frame_type, k210_frame_len, k210_payload);
                }

                K210_ResetParser();
                break;
            }

            default:
            {
                K210_ResetParser();
                break;
            }
        }
    }

    now = GetTick();
    K210_UpdateTimeout(now);
}

/* ==================== 内部函数实现 ==================== */
static void K210_ResetParser(void)
{
    k210_state = K210_WAIT_HEAD1;
    k210_frame_type = 0;
    k210_frame_len = 0;
    k210_frame_idx = 0;
    k210_checksum = 0;
    memset(k210_payload, 0, sizeof(k210_payload));
}

static int16_t K210_ReadInt16LE(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static void K210_ProcessFrame(uint8_t type, uint8_t len, const uint8_t *payload)
{
    if (payload == 0)
    {
        return;
    }

    switch (type)
    {
        case K210_FRAME_TYPE_LINE:
        {
            /* 巡线偏差：payload[0..1] = int16_t line_offset */
            if (len == 2)
            {
                k210_data.line_offset = K210_ReadInt16LE(payload);
                k210_data.line_valid = 1;
                k210_data.line_stamp = GetTick();
            }
            break;
        }

        case K210_FRAME_TYPE_QR:
        {
            /* 二维码中心：payload[0..1] = int16_t qr_x, payload[2..3] = int16_t qr_y */
            if (len == 4)
            {
                k210_data.qr_x = K210_ReadInt16LE(&payload[0]);
                k210_data.qr_y = K210_ReadInt16LE(&payload[2]);
                k210_data.qr_valid = 1;
                k210_data.qr_stamp = GetTick();
            }
            break;
        }

        default:
        {
            /* 未知帧类型，忽略 */
            break;
        }
    }
}

static void K210_UpdateTimeout(uint32_t now)
{
    if (k210_data.line_valid)
    {
        if ((now - k210_data.line_stamp) > K210_LINE_TIMEOUT_MS)
        {
            k210_data.line_valid = 0;
        }
    }

    if (k210_data.qr_valid)
    {
        if ((now - k210_data.qr_stamp) > K210_QR_TIMEOUT_MS)
        {
            k210_data.qr_valid = 0;
        }
    }
}
