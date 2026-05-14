#include "bsp_spi.h"

#define SPI1_DMA_RX_STREAM      DMA2_Stream0
#define SPI1_DMA_TX_STREAM      DMA2_Stream3
#define SPI1_DMA_CHANNEL        DMA_Channel_3

#define SPI1_DMA_RX_IRQn        DMA2_Stream0_IRQn
#define SPI1_DMA_TX_IRQn        DMA2_Stream3_IRQn

#define SPI1_BLOCK_TIMEOUT      100000UL
#define SPI1_DMA_DISABLE_TIMEOUT 100000UL

static volatile uint8_t spi1_busy = 0;
static volatile uint8_t spi1_cb_called = 0;
static spi1_done_cb_t spi1_done_cb = 0;
static void *spi1_done_ctx = 0;

static uint8_t SPI1_WaitDisable(DMA_Stream_TypeDef *stream)
{
    uint32_t timeout = SPI1_DMA_DISABLE_TIMEOUT;

    while (DMA_GetCmdStatus(stream) != DISABLE) {
        if (--timeout == 0U) {
            return 0;
        }
    }
    return 1;
}

static void SPI1_FinishFromISR(uint8_t status)
{
    spi1_done_cb_t cb;
    void *ctx;

    if (spi1_cb_called) {
        return;
    }

    spi1_cb_called = 1;

    SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Rx, DISABLE);
    SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, DISABLE);

    DMA_Cmd(SPI1_DMA_RX_STREAM, DISABLE);
    DMA_Cmd(SPI1_DMA_TX_STREAM, DISABLE);

    cb = spi1_done_cb;
    ctx = spi1_done_ctx;

    spi1_done_cb = 0;
    spi1_done_ctx = 0;
    spi1_busy = 0;

    if (cb != 0) {
        cb(ctx, status);
    }
}

static void SPI1_DMA_Init(void)
{
    DMA_InitTypeDef DMA_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

    DMA_DeInit(SPI1_DMA_RX_STREAM);
    (void)SPI1_WaitDisable(SPI1_DMA_RX_STREAM);

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
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(SPI1_DMA_RX_STREAM, &DMA_InitStructure);
    DMA_ITConfig(SPI1_DMA_RX_STREAM, DMA_IT_TC | DMA_IT_TE | DMA_IT_DME | DMA_IT_FE, ENABLE);

    DMA_DeInit(SPI1_DMA_TX_STREAM);
    (void)SPI1_WaitDisable(SPI1_DMA_TX_STREAM);

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
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(SPI1_DMA_TX_STREAM, &DMA_InitStructure);
    DMA_ITConfig(SPI1_DMA_TX_STREAM, DMA_IT_TE | DMA_IT_DME | DMA_IT_FE, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = SPI1_DMA_RX_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = SPI1_DMA_TX_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
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

    /* PMW3901: SPI Mode 3 */
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;

    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;

    /*
     * 调试阶段先用 /128，稳定后可改 /64。
     * SPI1 在 APB2=84MHz 时：
     *   /128 = 656 kHz
     *   /64  = 1.3125 MHz
     */
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;

    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init(SPI1, &SPI_InitStructure);

    SPI_NSSInternalSoftwareConfig(SPI1, SPI_NSSInternalSoft_Set);

    SPI1_DMA_Init();

    SPI_Cmd(SPI1, ENABLE);

    spi1_busy = 0;
    spi1_cb_called = 0;
    spi1_done_cb = 0;
    spi1_done_ctx = 0;
}

uint8_t SPI1_TransferByte(uint8_t tx, uint8_t *rx)
{
    uint32_t timeout;

    if (rx == 0) {
        return 0;
    }

    timeout = SPI1_BLOCK_TIMEOUT;
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET) {
        if (--timeout == 0U) {
            return 0;
        }
    }

    SPI_I2S_SendData(SPI1, tx);

    timeout = SPI1_BLOCK_TIMEOUT;
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET) {
        if (--timeout == 0U) {
            return 0;
        }
    }

    *rx = (uint8_t)SPI_I2S_ReceiveData(SPI1);
    return 1;
}

uint8_t SPI1_ReadWriteByte(uint8_t data)
{
    uint8_t rx = 0xFF;

    (void)SPI1_TransferByte(data, &rx);
    return rx;
}

uint8_t SPI1_TransferAsync_DMA(uint8_t *tx_buf,
                               uint8_t *rx_buf,
                               uint16_t len,
                               spi1_done_cb_t cb,
                               void *ctx)
{
    uint8_t dummy;
    DMA_InitTypeDef DMA_InitStructure;
    uint32_t primask;

    if (len == 0 || tx_buf == 0 || rx_buf == 0) {
        return 0;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    if (spi1_busy) {
        if (!primask) __enable_irq();
        return 0;
    }

    spi1_busy = 1;
    spi1_cb_called = 0;
    spi1_done_cb = cb;
    spi1_done_ctx = ctx;

    if (!primask) __enable_irq();

    SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Rx, DISABLE);
    SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, DISABLE);

    DMA_Cmd(SPI1_DMA_RX_STREAM, DISABLE);
    DMA_Cmd(SPI1_DMA_TX_STREAM, DISABLE);

    if (!SPI1_WaitDisable(SPI1_DMA_RX_STREAM) ||
        !SPI1_WaitDisable(SPI1_DMA_TX_STREAM)) {
        SPI1_FinishFromISR(SPI1_STATUS_TIMEOUT);
        return 0;
    }

    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == SET) {
        dummy = (uint8_t)SPI_I2S_ReceiveData(SPI1);
        (void)dummy;
    }

    DMA_ClearFlag(SPI1_DMA_RX_STREAM,
                  DMA_FLAG_TCIF0 | DMA_FLAG_HTIF0 |
                  DMA_FLAG_TEIF0 | DMA_FLAG_DMEIF0 | DMA_FLAG_FEIF0);

    DMA_ClearFlag(SPI1_DMA_TX_STREAM,
                  DMA_FLAG_TCIF3 | DMA_FLAG_HTIF3 |
                  DMA_FLAG_TEIF3 | DMA_FLAG_DMEIF3 | DMA_FLAG_FEIF3);

    DMA_DeInit(SPI1_DMA_RX_STREAM);
    if (!SPI1_WaitDisable(SPI1_DMA_RX_STREAM)) {
        SPI1_FinishFromISR(SPI1_STATUS_TIMEOUT);
        return 0;
    }

    DMA_InitStructure.DMA_Channel = SPI1_DMA_CHANNEL;
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
    DMA_Init(SPI1_DMA_RX_STREAM, &DMA_InitStructure);
    DMA_ITConfig(SPI1_DMA_RX_STREAM, DMA_IT_TC | DMA_IT_TE | DMA_IT_DME | DMA_IT_FE, ENABLE);

    DMA_DeInit(SPI1_DMA_TX_STREAM);
    if (!SPI1_WaitDisable(SPI1_DMA_TX_STREAM)) {
        SPI1_FinishFromISR(SPI1_STATUS_TIMEOUT);
        return 0;
    }

    DMA_InitStructure.DMA_Channel = SPI1_DMA_CHANNEL;
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
    DMA_Init(SPI1_DMA_TX_STREAM, &DMA_InitStructure);
    DMA_ITConfig(SPI1_DMA_TX_STREAM, DMA_IT_TE | DMA_IT_DME | DMA_IT_FE, ENABLE);

    DMA_Cmd(SPI1_DMA_RX_STREAM, ENABLE);
    DMA_Cmd(SPI1_DMA_TX_STREAM, ENABLE);

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
    if (DMA_GetITStatus(SPI1_DMA_RX_STREAM, DMA_IT_TEIF0) != RESET ||
        DMA_GetITStatus(SPI1_DMA_RX_STREAM, DMA_IT_DMEIF0) != RESET ||
        DMA_GetITStatus(SPI1_DMA_RX_STREAM, DMA_IT_FEIF0) != RESET)
    {
        DMA_ClearITPendingBit(SPI1_DMA_RX_STREAM,
                              DMA_IT_TEIF0 | DMA_IT_DMEIF0 | DMA_IT_FEIF0);
        SPI1_FinishFromISR(SPI1_STATUS_ERROR);
        return;
    }

    if (DMA_GetITStatus(SPI1_DMA_RX_STREAM, DMA_IT_TCIF0) != RESET)
    {
        DMA_ClearITPendingBit(SPI1_DMA_RX_STREAM, DMA_IT_TCIF0);

        while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) == SET) {
            ;
        }

        SPI1_FinishFromISR(SPI1_STATUS_OK);
    }
}

void SPI1_DMA_IRQHandler_TX(void)
{
    if (DMA_GetITStatus(SPI1_DMA_TX_STREAM, DMA_IT_TEIF3) != RESET ||
        DMA_GetITStatus(SPI1_DMA_TX_STREAM, DMA_IT_DMEIF3) != RESET ||
        DMA_GetITStatus(SPI1_DMA_TX_STREAM, DMA_IT_FEIF3) != RESET)
    {
        DMA_ClearITPendingBit(SPI1_DMA_TX_STREAM,
                              DMA_IT_TEIF3 | DMA_IT_DMEIF3 | DMA_IT_FEIF3);
        SPI1_FinishFromISR(SPI1_STATUS_ERROR);
    }
}
