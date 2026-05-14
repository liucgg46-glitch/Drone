#include "bsp_spi.h"

#define SPI1_DMA_RX_STREAM      DMA2_Stream0
#define SPI1_DMA_TX_STREAM      DMA2_Stream3
#define SPI1_DMA_CHANNEL        DMA_Channel_3

#define SPI1_DMA_RX_IRQn        DMA2_Stream0_IRQn
#define SPI1_DMA_TX_IRQn        DMA2_Stream3_IRQn

static volatile uint8_t spi1_busy = 0;
static spi1_done_cb_t spi1_done_cb = 0;
static void *spi1_done_ctx = 0;

static void SPI1_DMA_Init(void)
{
    DMA_InitTypeDef DMA_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

    /* ================= RX: DMA2 Stream0 Channel3 ================= */
    DMA_DeInit(SPI1_DMA_RX_STREAM);
    while (DMA_GetCmdStatus(SPI1_DMA_RX_STREAM) != DISABLE);

    DMA_InitStructure.DMA_Channel = SPI1_DMA_CHANNEL;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&SPI1->DR;
    DMA_InitStructure.DMA_Memory0BaseAddr = 0;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize = 0;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(SPI1_DMA_RX_STREAM, &DMA_InitStructure);

    DMA_ITConfig(SPI1_DMA_RX_STREAM, DMA_IT_TC | DMA_IT_TE | DMA_IT_DME | DMA_IT_FE, ENABLE);

    /* ================= TX: DMA2 Stream3 Channel3 ================= */
    DMA_DeInit(SPI1_DMA_TX_STREAM);
    while (DMA_GetCmdStatus(SPI1_DMA_TX_STREAM) != DISABLE);

    DMA_InitStructure.DMA_Channel = SPI1_DMA_CHANNEL;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&SPI1->DR;
    DMA_InitStructure.DMA_Memory0BaseAddr = 0;
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    DMA_InitStructure.DMA_BufferSize = 0;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(SPI1_DMA_TX_STREAM, &DMA_InitStructure);

    DMA_ITConfig(SPI1_DMA_TX_STREAM, DMA_IT_TE | DMA_IT_DME | DMA_IT_FE, ENABLE);

    /* RX DMA IRQ */
    NVIC_InitStructure.NVIC_IRQChannel = SPI1_DMA_RX_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* TX DMA IRQ */
    NVIC_InitStructure.NVIC_IRQChannel = SPI1_DMA_TX_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void SPI1_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    SPI_InitTypeDef SPI_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    SPI_I2S_DeInit(SPI1);

    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;

    /* PMW3901 使用 SPI Mode 3 */
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;

    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;

    SPI_Init(SPI1, &SPI_InitStructure);

    SPI_NSSInternalSoftwareConfig(SPI1, SPI_NSSInternalSoft_Set);

    SPI1_DMA_Init();

    SPI_Cmd(SPI1, ENABLE);

    spi1_busy = 0;
}

/* 原来的阻塞接口保留 */
uint8_t SPI1_ReadWriteByte(uint8_t data)
{
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
    SPI_I2S_SendData(SPI1, data);

    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
    return (uint8_t)SPI_I2S_ReceiveData(SPI1);
}

/* 新增：SPI1 DMA 无阻塞收发 */
uint8_t SPI1_TransferAsync_DMA(uint8_t *tx_buf,
                               uint8_t *rx_buf,
                               uint16_t len,
                               spi1_done_cb_t cb,
                               void *ctx)
{
    uint8_t dummy;
    DMA_InitTypeDef DMA_InitStructure;

    if (len == 0 || tx_buf == 0 || rx_buf == 0) {
        return 0;
    }

    if (spi1_busy) {
        return 0;
    }

    spi1_busy = 1;
    spi1_done_cb = cb;
    spi1_done_ctx = ctx;

    /* 先关 SPI DMA 请求 */
    SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Rx, DISABLE);
    SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, DISABLE);

    /* 关闭 DMA Stream */
    DMA_Cmd(DMA2_Stream0, DISABLE);
    DMA_Cmd(DMA2_Stream3, DISABLE);

    while (DMA_GetCmdStatus(DMA2_Stream0) != DISABLE);
    while (DMA_GetCmdStatus(DMA2_Stream3) != DISABLE);

    /* 清 SPI 残留 RXNE */
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == SET) {
        dummy = (uint8_t)SPI_I2S_ReceiveData(SPI1);
        (void)dummy;
    }

    /* 清 DMA 标志 */
    DMA_ClearFlag(DMA2_Stream0,
                  DMA_FLAG_TCIF0 | DMA_FLAG_HTIF0 |
                  DMA_FLAG_TEIF0 | DMA_FLAG_DMEIF0 | DMA_FLAG_FEIF0);

    DMA_ClearFlag(DMA2_Stream3,
                  DMA_FLAG_TCIF3 | DMA_FLAG_HTIF3 |
                  DMA_FLAG_TEIF3 | DMA_FLAG_DMEIF3 | DMA_FLAG_FEIF3);

    /*
     * 关键修改：
     * 每次传输前重新初始化 DMA。
     * 不要只改 M0AR 和 NDTR。
     */

    /* ================= RX: DMA2 Stream0 Channel3 ================= */
    DMA_DeInit(DMA2_Stream0);
    while (DMA_GetCmdStatus(DMA2_Stream0) != DISABLE);

    DMA_InitStructure.DMA_Channel = DMA_Channel_3;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&SPI1->DR;
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)rx_buf;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize = len;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

    DMA_Init(DMA2_Stream0, &DMA_InitStructure);

    DMA_ITConfig(DMA2_Stream0,
                 DMA_IT_TC | DMA_IT_TE | DMA_IT_DME | DMA_IT_FE,
                 ENABLE);

    /* ================= TX: DMA2 Stream3 Channel3 ================= */
    DMA_DeInit(DMA2_Stream3);
    while (DMA_GetCmdStatus(DMA2_Stream3) != DISABLE);

    DMA_InitStructure.DMA_Channel = DMA_Channel_3;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&SPI1->DR;
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)tx_buf;
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    DMA_InitStructure.DMA_BufferSize = len;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

    DMA_Init(DMA2_Stream3, &DMA_InitStructure);

    DMA_ITConfig(DMA2_Stream3,
                 DMA_IT_TE | DMA_IT_DME | DMA_IT_FE,
                 ENABLE);

    /*
     * 关键顺序：
     * 1. 先开 RX Stream
     * 2. 再开 TX Stream
     * 3. 最后开 SPI DMA 请求
     */
    DMA_Cmd(DMA2_Stream0, ENABLE);
    DMA_Cmd(DMA2_Stream3, ENABLE);

    SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Rx, ENABLE);
    SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, ENABLE);

    return 1;
}
uint8_t SPI1_IsBusy(void)
{
    return spi1_busy;
}

void SPI1_DMA_IRQHandler_RX(void)
{
    uint8_t status = 0;

    if (DMA_GetITStatus(DMA2_Stream0, DMA_IT_TEIF0) != RESET ||
        DMA_GetITStatus(DMA2_Stream0, DMA_IT_DMEIF0) != RESET ||
        DMA_GetITStatus(DMA2_Stream0, DMA_IT_FEIF0) != RESET)
    {
        DMA_ClearITPendingBit(DMA2_Stream0,
                              DMA_IT_TEIF0 | DMA_IT_DMEIF0 | DMA_IT_FEIF0);

        status = 1;
    }

    if (DMA_GetITStatus(DMA2_Stream0, DMA_IT_TCIF0) != RESET)
    {
        DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_TCIF0);

        while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) == SET);

        SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Rx, DISABLE);
        SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, DISABLE);

        DMA_Cmd(DMA2_Stream0, DISABLE);
        DMA_Cmd(DMA2_Stream3, DISABLE);

        spi1_busy = 0;

        if (spi1_done_cb) {
            spi1_done_cb(spi1_done_ctx, status);
        }
    }
}

/* TX 中断只处理错误，不在这里判定完成 */
void SPI1_DMA_IRQHandler_TX(void)
{
    if (DMA_GetITStatus(DMA2_Stream3, DMA_IT_TEIF3) != RESET ||
        DMA_GetITStatus(DMA2_Stream3, DMA_IT_DMEIF3) != RESET ||
        DMA_GetITStatus(DMA2_Stream3, DMA_IT_FEIF3) != RESET)
    {
        DMA_ClearITPendingBit(DMA2_Stream3,
                              DMA_IT_TEIF3 | DMA_IT_DMEIF3 | DMA_IT_FEIF3);

        SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Rx, DISABLE);
        SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, DISABLE);

        DMA_Cmd(DMA2_Stream0, DISABLE);
        DMA_Cmd(DMA2_Stream3, DISABLE);

        spi1_busy = 0;

        if (spi1_done_cb) {
            spi1_done_cb(spi1_done_ctx, 1);
        }
    }
}
