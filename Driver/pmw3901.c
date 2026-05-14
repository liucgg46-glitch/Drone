#include "pmw3901.h"
#include "bsp_spi.h"

#define PMW3901_CS_LOW()      GPIO_ResetBits(GPIOB, GPIO_Pin_0)
#define PMW3901_CS_HIGH()     GPIO_SetBits(GPIOB, GPIO_Pin_0)

#define PMW3901_RST_LOW()     GPIO_ResetBits(GPIOB, GPIO_Pin_1)
#define PMW3901_RST_HIGH()    GPIO_SetBits(GPIOB, GPIO_Pin_1)

static void PMW3901_Delay(volatile uint32_t t)
{
    while (t--);
}

static void PMW3901_DelayMs(uint32_t ms)
{
    while (ms--)
    {
        PMW3901_Delay(84000);
    }
}

void PMW3901_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    PMW3901_CS_HIGH();
    PMW3901_RST_HIGH();
}

uint8_t PMW3901_ReadReg(uint8_t reg)
{
    uint8_t data;

    PMW3901_CS_LOW();
    PMW3901_Delay(100);

    SPI1_ReadWriteByte(reg & 0x7F);

    PMW3901_Delay(100);

    data = SPI1_ReadWriteByte(0x00);

    PMW3901_CS_HIGH();

    PMW3901_Delay(100);

    return data;
}

void PMW3901_WriteReg(uint8_t reg, uint8_t data)
{
    PMW3901_CS_LOW();
    PMW3901_Delay(100);

    SPI1_ReadWriteByte(reg | 0x80);
    SPI1_ReadWriteByte(data);

    PMW3901_CS_HIGH();

    PMW3901_Delay(100);
}

uint8_t PMW3901_ReadID(void)
{
    return PMW3901_ReadReg(PMW3901_PRODUCT_ID);
}

uint8_t PMW3901_Init(void)
{
    uint8_t id;

    PMW3901_GPIO_Init();
    SPI1_Init();

    PMW3901_CS_HIGH();

    PMW3901_RST_LOW();
    PMW3901_DelayMs(10);
    PMW3901_RST_HIGH();
    PMW3901_DelayMs(50);

    id = PMW3901_ReadID();

    if (id != 0x49)
    {
        return 0;
    }

    /* Power-up reset */
    PMW3901_WriteReg(0x3A, 0x5A);
    PMW3901_DelayMs(5);

    /* 清除一次运动寄存器 */
    PMW3901_ReadReg(0x02);
    PMW3901_ReadReg(0x03);
    PMW3901_ReadReg(0x04);
    PMW3901_ReadReg(0x05);
    PMW3901_ReadReg(0x06);

    /* PMW3901 初始化寄存器序列 */
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
    PMW3901_ReadReg(0x67);
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

    PMW3901_DelayMs(10);

    return 1;
}

uint8_t PMW3901_ReadMotion(pmw3901_t *flow)
{
    uint8_t buf[12];
    uint8_t i;

    if (flow == 0)
        return 0;

    PMW3901_CS_LOW();
    PMW3901_Delay(100);

    SPI1_ReadWriteByte(PMW3901_MOTION_BURST & 0x7F);

    PMW3901_Delay(500);

    for (i = 0; i < 12; i++)
    {
        buf[i] = SPI1_ReadWriteByte(0x00);
    }

    PMW3901_CS_HIGH();
    PMW3901_Delay(100);

    flow->motion = buf[0];

    flow->dx = (int16_t)((buf[3] << 8) | buf[2]);
    flow->dy = (int16_t)((buf[5] << 8) | buf[4]);

    flow->quality = buf[6];

    return 1;
}
/* ================= PMW3901 Async Motion Burst ================= */

extern uint32_t GetTick(void);

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

static uint32_t pmw3901_wait_tick = 0;

static void PMW3901_DMA_DoneCallback(void *ctx, uint8_t status)
{
    (void)ctx;

    /*
     * 注意：这是在 DMA 中断里调用的。
     * 中断里不要继续启动下一次 SPI，只置标志。
     */
    pmw3901_dma_status = status;
    pmw3901_dma_done = 1;
}

uint8_t PMW3901_ReadMotion_Async(pmw3901_t *flow)
{
    if (flow == 0) {
        return 0;
    }

    if (pmw3901_async_state != PMW3901_ASYNC_IDLE &&
        pmw3901_async_state != PMW3901_ASYNC_DONE &&
        pmw3901_async_state != PMW3901_ASYNC_ERROR)
    {
        return 0;
    }

    if (SPI1_IsBusy()) {
        return 0;
    }

    pmw3901_async_flow = flow;
    pmw3901_motion_done = 0;
    pmw3901_dma_done = 0;
    pmw3901_dma_status = 0;

    pmw3901_cmd_tx[0] = PMW3901_MOTION_BURST & 0x7F;
    pmw3901_cmd_rx[0] = 0x00;

    PMW3901_CS_LOW();

    /*
     * 严格无阻塞：
     * 这里不再用 SPI1_ReadWriteByte()
     * 也不再用 PMW3901_Delay()
     * 命令字 0x16 也走 DMA 异步发送。
     */
    if (SPI1_TransferAsync_DMA(pmw3901_cmd_tx,
                               pmw3901_cmd_rx,
                               1,
                               PMW3901_DMA_DoneCallback,
                               0) == 0)
    {
        PMW3901_CS_HIGH();
        pmw3901_async_state = PMW3901_ASYNC_ERROR;
        return 0;
    }

    pmw3901_async_state = PMW3901_ASYNC_CMD_BUSY;

    return 1;
}
void PMW3901_Task(void)
{
    uint8_t i;

    switch (pmw3901_async_state)
    {
        case PMW3901_ASYNC_CMD_BUSY:
        {
            if (pmw3901_dma_done)
            {
                pmw3901_dma_done = 0;

                if (pmw3901_dma_status != 0)
                {
                    PMW3901_CS_HIGH();
                    pmw3901_async_state = PMW3901_ASYNC_ERROR;
                    break;
                }

                /*
                 * 命令 0x16 已经发完。
                 * 不用 PMW3901_Delay(500)，改成 GetTick 非阻塞等待。
                 * 先用 1ms，稳定优先。
                 */
                pmw3901_wait_tick = GetTick();
                pmw3901_async_state = PMW3901_ASYNC_WAIT_GAP;
            }
            break;
        }

        case PMW3901_ASYNC_WAIT_GAP:
        {
            /*
             * 非阻塞等待 burst 命令和数据读取之间的间隔。
             * 这里不会卡住 CPU，主循环可以继续跑其他任务。
             */
            if ((GetTick() - pmw3901_wait_tick) >= 1)
            {
                if (SPI1_IsBusy()) {
                    break;
                }

                for (i = 0; i < 12; i++) {
                    pmw3901_dummy_tx[i] = 0x00;
                    pmw3901_burst_rx[i] = 0x00;
                }

                pmw3901_dma_done = 0;
                pmw3901_dma_status = 0;

                /*
                 * DMA 异步读取 12 字节 motion burst 数据。
                 */
                if (SPI1_TransferAsync_DMA(pmw3901_dummy_tx,
                                           pmw3901_burst_rx,
                                           12,
                                           PMW3901_DMA_DoneCallback,
                                           0) == 0)
                {
                    PMW3901_CS_HIGH();
                    pmw3901_async_state = PMW3901_ASYNC_ERROR;
                }
                else
                {
                    pmw3901_async_state = PMW3901_ASYNC_READ_BUSY;
                }
            }
            break;
        }

        case PMW3901_ASYNC_READ_BUSY:
        {
            if (pmw3901_dma_done)
            {
                pmw3901_dma_done = 0;

                PMW3901_CS_HIGH();

                if (pmw3901_dma_status != 0)
                {
                    pmw3901_async_state = PMW3901_ASYNC_ERROR;
                    break;
                }

                if (pmw3901_async_flow != 0)
                {
                    pmw3901_async_flow->motion = pmw3901_burst_rx[0];

                    pmw3901_async_flow->dx =
                        (int16_t)((pmw3901_burst_rx[3] << 8) |
                                   pmw3901_burst_rx[2]);

                    pmw3901_async_flow->dy =
                        (int16_t)((pmw3901_burst_rx[5] << 8) |
                                   pmw3901_burst_rx[4]);

                    pmw3901_async_flow->quality = pmw3901_burst_rx[6];
                }

                pmw3901_motion_done = 1;
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
        pmw3901_motion_done = 0;
        pmw3901_async_state = PMW3901_ASYNC_IDLE;
        return 1;
    }

    return 0;
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

    for (i = 0; i < 12; i++) {
        buf[i] = pmw3901_burst_rx[i];
    }
}
