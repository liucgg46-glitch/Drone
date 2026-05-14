#ifndef __K210_PROTOCOL_H
#define __K210_PROTOCOL_H

#include <stdint.h>

/* ==================== K210 协议定义 ==================== */
#define K210_FRAME_HEAD1            0xAA
#define K210_FRAME_HEAD2            0x55

#define K210_FRAME_TYPE_LINE        0x01    /* 巡线偏差帧 */
#define K210_FRAME_TYPE_QR          0x02    /* 二维码中心帧 */

/* payload 最大长度，按当前协议足够用 */
#define K210_FRAME_MAX_PAYLOAD      16

/* 数据超时判定 */
#define K210_LINE_TIMEOUT_MS        200
#define K210_QR_TIMEOUT_MS          500

/* ==================== K210 数据结构 ==================== */
typedef struct
{
    int16_t line_offset;     /* 巡线偏差，正负表示左右偏移 */
    int16_t qr_x;            /* 二维码中心 X */
    int16_t qr_y;            /* 二维码中心 Y */

    uint8_t line_valid;      /* 巡线数据是否有效 */
    uint8_t qr_valid;        /* 二维码数据是否有效 */

    uint32_t line_stamp;     /* 巡线数据时间戳 */
    uint32_t qr_stamp;       /* 二维码数据时间戳 */
} k210_data_t;

/* ==================== 对外接口 ==================== */
void K210_Init(void);
void K210_ClearData(void);
void K210_ParseTask(void);

const k210_data_t *K210_GetData(void);

int16_t K210_GetLineOffset(void);
int16_t K210_GetQrX(void);
int16_t K210_GetQrY(void);

uint8_t K210_LineAvailable(void);
uint8_t K210_QRAvailable(void);

#endif

