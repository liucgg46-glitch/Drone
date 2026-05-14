#include "pmw3901.h"
#include "bsp_spi.h"
#include "bsp_timer.h"
#include <string.h>

#define PMW3901_CS_LOW()      GPIO_ResetBits(PMW3901_CS_PORT, PMW3901_CS_PIN)
#define PMW3901_CS_HIGH()     GPIO_SetBits(PMW3901_CS_PORT, PMW3901_CS_PIN)

#define PMW3901_RST_LOW()     GPIO_ResetBits(PMW3901_RST_PORT, PMW3901_RST_PIN)
#define PMW3901_RST_HIGH()    GPIO_SetBits(PMW3901_RST_PORT, PMW3901_RST_PIN)

/* 粗略 us 延时。F407 168MHz 下偏保守，调传感器足够。 */
static void PMW3901_DelayCycle(volatile uint32_t t)
{
    while (t--) {
        __NOP();
    }
}

static void PMW3901_DelayUs(uint32_t us)
{
    while (us--) {
        PMW3901_DelayCycle(28U);
    }
}

static void PMW3901_DelayMs_Blocking(uint32_t ms)
{
    while (ms--) {
        PMW3901_DelayUs(1000U);
    }
}

void PMW3901_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(PMW3901_GPIO_RCC, ENABLE);

    GPIO_InitStructure.GPIO_Pin = PMW3901_CS_PIN | PMW3901_RST_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(PMW3901_CS_PORT, &GPIO_InitStructure);

    PMW3901_CS_HIGH();
    PMW3901_RST_HIGH();
}

uint8_t PMW3901_ReadReg(uint8_t reg)
{
    uint8_t data = 0xFF;

    PMW3901_CS_LOW();
    PMW3901_DelayUs(5U);

    (void)SPI1_ReadWriteByte(reg & 0x7FU);

    /* PMW3901 tSRAD 保守拉长，避免读到 0xFF */
    PMW3901_DelayUs(160U);

    data = SPI1_ReadWriteByte(0x00U);

    PMW3901_CS_HIGH();

    /* tSRW / tSRR 保守间隔 */
    PMW3901_DelayUs(30U);

    return data;
}

void PMW3901_WriteReg(uint8_t reg, uint8_t data)
{
    PMW3901_CS_LOW();
    PMW3901_DelayUs(5U);

    (void)SPI1_ReadWriteByte(reg | 0x80U);
    (void)SPI1_ReadWriteByte(data);

    PMW3901_CS_HIGH();

    /* 写寄存器后需要间隔，调试阶段保守一点 */
    PMW3901_DelayUs(60U);
}

uint8_t PMW3901_ReadID(void)
{
    return PMW3901_ReadReg(PMW3901_REG_PRODUCT_ID);
}

uint8_t PMW3901_ReadRevisionID(void)
{
    return PMW3901_ReadReg(PMW3901_REG_REVISION_ID);
}

/* PixArt/开源驱动常见初始化序列，先照你给的设计保留。 */
static void PMW3901_LoadInitRegisters(void)
{
    PMW3901_WriteReg(0x7F, 0x00);
    PMW3901_WriteReg(0x55, 0x01);
    PMW3901_WriteReg(0x50, 0x07);

    PMW3901_WriteReg(0x7F, 0x0E);
    PMW3901_WriteReg(0x43, 0x10);

    PMW3901_WriteReg(0x7F, 0x00);
    PMW3901_WriteReg(0x51, 0x7B);
    PMW3901_WriteReg(0x50, 0x00);
    PMW3901_WriteReg(0x55, 0x00);

    PMW3901_WriteReg(0x7F, 0x0E);
    (void)PMW3901_ReadReg(0x67);
    PMW3901_WriteReg(0x48, 0x0C);
    PMW3901_WriteReg(0x6F, 0x06);

    PMW3901_WriteReg(0x7F, 0x00);
    PMW3901_WriteReg(0x5B, 0xA0);
    PMW3901_WriteReg(0x4E, 0xA8);
    PMW3901_WriteReg(0x5A, 0x50);
    PMW3901_WriteReg(0x40, 0x80);

    PMW3901_WriteReg(0x7F, 0x00);
    PMW3901_WriteReg(0x5A, 0x10);
    PMW3901_WriteReg(0x54, 0x00);

    PMW3901_WriteReg(0x7F, 0x0E);
    PMW3901_WriteReg(0x41, 0xB3);
    PMW3901_WriteReg(0x43, 0xF1);
    PMW3901_WriteReg(0x45, 0x14);
    PMW3901_WriteReg(0x5F, 0x34);
    PMW3901_WriteReg(0x7B, 0x08);

    PMW3901_WriteReg(0x7F, 0x00);
    PMW3901_WriteReg(0x5B, 0x80);
    PMW3901_WriteReg(0x5C, 0x80);
    PMW3901_WriteReg(0x5D, 0x80);
    PMW3901_WriteReg(0x5E, 0x80);

    PMW3901_WriteReg(0x7F, 0x14);
    PMW3901_WriteReg(0x6A, 0x18);
    PMW3901_WriteReg(0x6B, 0x18);
    PMW3901_WriteReg(0x6C, 0x18);
    PMW3901_WriteReg(0x6D, 0x18);

    PMW3901_WriteReg(0x7F, 0x00);
    PMW3901_WriteReg(0x7F, 0x00);
}

uint8_t PMW3901_Init(void)
{
    uint8_t id;

    PMW3901_GPIO_Init();
    SPI1_Init();

    PMW3901_CS_HIGH();

    PMW3901_RST_LOW();
    PMW3901_DelayMs_Blocking(10U);
    PMW3901_RST_HIGH();
    PMW3901_DelayMs_Blocking(50U);

    id = PMW3901_ReadID();
    if (id != PMW3901_PRODUCT_ID_VALUE) {
        return PMW3901_FAIL;
    }

    /* Power-up reset */
    PMW3901_WriteReg(PMW3901_REG_POWER_UP_RESET, 0x5AU);
    PMW3901_DelayMs_Blocking(5U);

    /* 清一次 motion 寄存器 */
    (void)PMW3901_ReadReg(PMW3901_REG_MOTION);
    (void)PMW3901_ReadReg(PMW3901_REG_DELTA_X_L);
    (void)PMW3901_ReadReg(PMW3901_REG_DELTA_X_H);
    (void)PMW3901_ReadReg(PMW3901_REG_DELTA_Y_L);
    (void)PMW3901_ReadReg(PMW3901_REG_DELTA_Y_H);

    PMW3901_LoadInitRegisters();

    PMW3901_DelayMs_Blocking(10U);

    return PMW3901_OK;
}

static void PMW3901_ParseBurst(const uint8_t *buf, pmw3901_t *flow)
{
    if (buf == 0 || flow == 0) {
        return;
    }

    flow->motion = buf[0];

    flow->dx = (int16_t)((((uint16_t)buf[3]) << 8) | (uint16_t)buf[2]);
    flow->dy = (int16_t)((((uint16_t)buf[5]) << 8) | (uint16_t)buf[4]);

    flow->quality = buf[6];

    flow->motion_detected = (buf[0] & 0x80U) ? 1U : 0U;
    flow->overflow = (buf[0] & 0x10U) ? 1U : 0U;

    flow->shutter = (uint16_t)((((uint16_t)buf[10]) << 8) | (uint16_t)buf[11]);

    flow->stamp_ms = GetTick();
    flow->valid = 1U;
}

static uint8_t PMW3901_RawAllFF(const uint8_t *buf, uint8_t len)
{
    uint8_t i;

    if (buf == 0) {
        return 1;
    }

    for (i = 0; i < len; i++) {
        if (buf[i] != 0xFFU) {
            return 0;
        }
    }

    return 1;
}

uint8_t PMW3901_ReadMotionBurst_Blocking(pmw3901_t *flow, uint8_t raw[12])
{
    uint8_t buf[12];
    uint8_t i;

    if (flow == 0) {
        return PMW3901_FAIL;
    }

    PMW3901_CS_LOW();
    PMW3901_DelayUs(5U);

    (void)SPI1_ReadWriteByte(PMW3901_REG_MOTION_BURST & 0x7FU);

    /* burst read 的 tSRAD 保守等待 */
    PMW3901_DelayUs(160U);

    for (i = 0; i < 12U; i++) {
        buf[i] = SPI1_ReadWriteByte(0x00U);
    }

    PMW3901_CS_HIGH();
    PMW3901_DelayUs(30U);

    if (raw != 0) {
        for (i = 0; i < 12U; i++) {
            raw[i] = buf[i];
        }
    }

    if (PMW3901_RawAllFF(buf, 12U)) {
        flow->valid = 0U;
        return PMW3901_FAIL;
    }

    PMW3901_ParseBurst(buf, flow);
    return PMW3901_OK;
}

uint8_t PMW3901_ReadMotion(pmw3901_t *flow)
{
    return PMW3901_ReadMotionBurst_Blocking(flow, 0);
}

/* ================= PMW3901 Async Motion Burst ================= */

typedef enum
{
    PMW3901_ASYNC_IDLE = 0,
    PMW3901_ASYNC_CMD_BUSY,
    PMW3901_ASYNC_WAIT_GAP,
    PMW3901_ASYNC_READ_BUSY,
    PMW3901_ASYNC_DONE,
    PMW3901_ASYNC_ERROR
} pmw3901_async_state_t;

static volatile pmw3901_async_state_t pmw3901_async_state = PMW3901_ASYNC_IDLE;

static pmw3901_t *pmw3901_async_flow = 0;

static uint8_t pmw3901_cmd_tx[1];
static uint8_t pmw3901_cmd_rx[1];

static uint8_t pmw3901_dummy_tx[12];
static uint8_t pmw3901_burst_rx[12];

static volatile uint8_t pmw3901_dma_done = 0;
static volatile uint8_t pmw3901_dma_status = 0;
static volatile uint8_t pmw3901_motion_done = 0;

static uint32_t pmw3901_state_tick = 0;

static void PMW3901_DMA_DoneCallback(void *ctx, uint8_t status)
{
    (void)ctx;

    /* DMA 中断上下文：只置标志，不做下一次传输，不打印 */
    pmw3901_dma_status = status;
    pmw3901_dma_done = 1U;
}

uint8_t PMW3901_ReadMotion_Async(pmw3901_t *flow)
{
    if (flow == 0) {
        return PMW3901_FAIL;
    }

    if (pmw3901_async_state != PMW3901_ASYNC_IDLE &&
        pmw3901_async_state != PMW3901_ASYNC_DONE &&
        pmw3901_async_state != PMW3901_ASYNC_ERROR)
    {
        return PMW3901_FAIL;
    }

    if (SPI1_IsBusy()) {
        return PMW3901_FAIL;
    }

    pmw3901_async_flow = flow;
    pmw3901_motion_done = 0U;
    pmw3901_dma_done = 0U;
    pmw3901_dma_status = 0U;

    pmw3901_cmd_tx[0] = PMW3901_REG_MOTION_BURST & 0x7FU;
    pmw3901_cmd_rx[0] = 0x00U;

    PMW3901_CS_LOW();
    PMW3901_DelayUs(5U);

    if (SPI1_TransferAsync_DMA(pmw3901_cmd_tx,
                               pmw3901_cmd_rx,
                               1U,
                               PMW3901_DMA_DoneCallback,
                               0) == 0U)
    {
        PMW3901_CS_HIGH();
        pmw3901_async_state = PMW3901_ASYNC_ERROR;
        return PMW3901_FAIL;
    }

    pmw3901_state_tick = GetTick();
    pmw3901_async_state = PMW3901_ASYNC_CMD_BUSY;

    return PMW3901_OK;
}

void PMW3901_Task(void)
{
    uint8_t i;

    switch (pmw3901_async_state)
    {
        case PMW3901_ASYNC_CMD_BUSY:
        {
            if ((GetTick() - pmw3901_state_tick) > 5U) {
                PMW3901_CS_HIGH();
                pmw3901_async_state = PMW3901_ASYNC_ERROR;
                break;
            }

            if (pmw3901_dma_done)
            {
                pmw3901_dma_done = 0U;

                if (pmw3901_dma_status != 0U)
                {
                    PMW3901_CS_HIGH();
                    pmw3901_async_state = PMW3901_ASYNC_ERROR;
                    break;
                }

                /*
                 * PMW3901 burst command 后需要等待 tSRAD。
                 * GetTick 只有 1ms 粒度，所以用 1ms，稳定优先。
                 */
                pmw3901_state_tick = GetTick();
                pmw3901_async_state = PMW3901_ASYNC_WAIT_GAP;
            }
            break;
        }

        case PMW3901_ASYNC_WAIT_GAP:
        {
            if ((GetTick() - pmw3901_state_tick) >= 1U)
            {
                if (SPI1_IsBusy()) {
                    break;
                }

                for (i = 0; i < 12U; i++) {
                    pmw3901_dummy_tx[i] = 0x00U;
                    pmw3901_burst_rx[i] = 0x00U;
                }

                pmw3901_dma_done = 0U;
                pmw3901_dma_status = 0U;

                if (SPI1_TransferAsync_DMA(pmw3901_dummy_tx,
                                           pmw3901_burst_rx,
                                           12U,
                                           PMW3901_DMA_DoneCallback,
                                           0) == 0U)
                {
                    PMW3901_CS_HIGH();
                    pmw3901_async_state = PMW3901_ASYNC_ERROR;
                }
                else
                {
                    pmw3901_state_tick = GetTick();
                    pmw3901_async_state = PMW3901_ASYNC_READ_BUSY;
                }
            }
            break;
        }

        case PMW3901_ASYNC_READ_BUSY:
        {
            if ((GetTick() - pmw3901_state_tick) > 5U) {
                PMW3901_CS_HIGH();
                pmw3901_async_state = PMW3901_ASYNC_ERROR;
                break;
            }

            if (pmw3901_dma_done)
            {
                pmw3901_dma_done = 0U;

                PMW3901_CS_HIGH();

                if (pmw3901_dma_status != 0U)
                {
                    pmw3901_async_state = PMW3901_ASYNC_ERROR;
                    break;
                }

                if (pmw3901_async_flow != 0) {
                    if (!PMW3901_RawAllFF(pmw3901_burst_rx, 12U)) {
                        PMW3901_ParseBurst(pmw3901_burst_rx, pmw3901_async_flow);
                    } else {
                        pmw3901_async_flow->valid = 0U;
                    }
                }

                pmw3901_motion_done = 1U;
                pmw3901_async_state = PMW3901_ASYNC_DONE;
            }
            break;
        }

        case PMW3901_ASYNC_DONE:
        case PMW3901_ASYNC_ERROR:
        case PMW3901_ASYNC_IDLE:
        default:
            break;
    }
}

uint8_t PMW3901_ReadMotion_IsDone(void)
{
    if (pmw3901_motion_done)
    {
        pmw3901_motion_done = 0U;
        pmw3901_async_state = PMW3901_ASYNC_IDLE;
        return 1U;
    }

    return 0U;
}

uint8_t PMW3901_IsBusy(void)
{
    return (pmw3901_async_state == PMW3901_ASYNC_CMD_BUSY ||
            pmw3901_async_state == PMW3901_ASYNC_WAIT_GAP ||
            pmw3901_async_state == PMW3901_ASYNC_READ_BUSY);
}

void PMW3901_GetAsyncRaw(uint8_t *buf)
{
    uint8_t i;

    if (buf == 0) {
        return;
    }

    for (i = 0; i < 12U; i++) {
        buf[i] = pmw3901_burst_rx[i];
    }
}
