#include "bsp_uart.h"
#include "stm32f4xx.h"
#include <string.h>

/* ---------- UART1 静态变量 ---------- */
static uint8_t uart1_tx_buf[UART_TX_BUF_SIZE];
static volatile uint16_t uart1_tx_wr = 0;
static volatile uint16_t uart1_tx_rd = 0;
static volatile uint8_t  uart1_tx_busy = 0;
static volatile uint16_t uart1_tx_len = 0;   // 当前 DMA 发送长度，用于回调更新读指针

static ring_buffer_t uart1_rx_buf = {{0}, 0, 0, 0};

uint8_t uart1_dma_rx_buf[UART_RX_BUF_SIZE];  // DMA 接收缓冲区

static void UART1_TxStart(void);

/* ---------- UART1 初始化 ---------- */
void UART1_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef  NVIC_InitStructure;
    DMA_InitTypeDef   DMA_InitStructure;

    /* 1) 时钟使能 */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

    /* 2) GPIO PA9(TX) PA10(RX) */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* 3) USART 配置：115200 8N1，无流控 */
    USART_InitStructure.USART_BaudRate            = 115200;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);

    /* 4) 使能空闲中断 */
    USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);

    /* 5) DMA 接收配置 (DMA2 Stream2 Channel4) */
    DMA_DeInit(DMA2_Stream2);
    DMA_InitStructure.DMA_Channel            = DMA_Channel_4;
    DMA_InitStructure.DMA_PeripheralBaseAddr  = (uint32_t)&USART1->DR;
    DMA_InitStructure.DMA_Memory0BaseAddr     = (uint32_t)uart1_dma_rx_buf;
    DMA_InitStructure.DMA_DIR                 = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize          = UART_RX_BUF_SIZE;
    DMA_InitStructure.DMA_PeripheralInc       = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc           = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize  = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize      = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode                = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority            = DMA_Priority_High;
    DMA_InitStructure.DMA_FIFOMode            = DMA_FIFOMode_Disable;
    DMA_Init(DMA2_Stream2, &DMA_InitStructure);

    /* 6) DMA 发送配置 (DMA2 Stream7 Channel4) */
    DMA_DeInit(DMA2_Stream7);
    DMA_InitStructure.DMA_Channel            = DMA_Channel_4;
    DMA_InitStructure.DMA_PeripheralBaseAddr  = (uint32_t)&USART1->DR;
    DMA_InitStructure.DMA_Memory0BaseAddr     = (uint32_t)uart1_tx_buf;
    DMA_InitStructure.DMA_DIR                 = DMA_DIR_MemoryToPeripheral;
    DMA_InitStructure.DMA_BufferSize          = UART_TX_BUF_SIZE;
    DMA_InitStructure.DMA_PeripheralInc       = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc           = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize  = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize      = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode                = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority            = DMA_Priority_High;
    DMA_InitStructure.DMA_FIFOMode            = DMA_FIFOMode_Disable;
    DMA_Init(DMA2_Stream7, &DMA_InitStructure);

    /* 7) 配置 NVIC：USART1 和 DMA2_Stream7 中断 */
    NVIC_InitStructure.NVIC_IRQChannel                   = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel                   = DMA2_Stream7_IRQn;
    NVIC_Init(&NVIC_InitStructure);

    /* 8) 使能 DMA 发送完成中断 */
    DMA_ITConfig(DMA2_Stream7, DMA_IT_TC, ENABLE);

    /* 9) 使能 DMA 请求 */
    USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
    USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);

    /* 10) 启动 DMA 接收 */
    DMA_Cmd(DMA2_Stream2, ENABLE);

    /* 11) 使能 USART */
    USART_Cmd(USART1, ENABLE);

    /* 12) 读一次 SR/DR 清除可能的空闲标志，避免上电后误进中断 */
    {
        uint32_t tmp;
        tmp = USART1->SR;
        tmp = USART1->DR;
        (void)tmp;
    }
}

/* ---------- 启动一次 TX DMA ---------- */
static void UART1_TxStart(void)
{
    uint16_t cnt = 0;
    uint16_t idx = uart1_tx_rd;

    if (uart1_tx_rd == uart1_tx_wr) {
        uart1_tx_busy = 0;
        return;
    }

    while (idx != uart1_tx_wr && cnt < UART_TX_BUF_SIZE) {
        cnt++;
        idx = (idx + 1) % UART_TX_BUF_SIZE;
    }

    if (cnt == 0) {
        uart1_tx_busy = 0;
        return;
    }

    /* 如果环形缓冲区跨边界，只发连续的第一段 */
    if ((uart1_tx_rd + cnt) > UART_TX_BUF_SIZE) {
        cnt = UART_TX_BUF_SIZE - uart1_tx_rd;
    }

    uart1_tx_len = cnt;

    /* 先关闭 DMA，再重设地址和长度 */
    DMA_Cmd(DMA2_Stream7, DISABLE);
    while (DMA_GetCmdStatus(DMA2_Stream7) != DISABLE) {
        ;
    }
    DMA2_Stream7->M0AR = (uint32_t)&uart1_tx_buf[uart1_tx_rd];
    DMA2_Stream7->NDTR = cnt;
    DMA_Cmd(DMA2_Stream7, ENABLE);

    uart1_tx_busy = 1;
}

/* ---------- 非阻塞发送 ---------- */
void UART1_SendData_NonBlocking(uint8_t *data, uint16_t len)
{
    uint16_t i;
    if (data == 0 || len == 0) {
        return;
    }

    __disable_irq();  // 进入临界区

    for (i = 0; i < len; i++) {
        uint16_t next_wr = (uart1_tx_wr + 1) % UART_TX_BUF_SIZE;
        if (next_wr != uart1_tx_rd) {  // 队列未满
            uart1_tx_buf[uart1_tx_wr] = data[i];
            uart1_tx_wr = next_wr;
        } else {
            /* 队列满则丢弃后续数据 */
            break;
        }
    }

    /* 如果发送空闲，立即启动一次发送 */
    if (!uart1_tx_busy && (uart1_tx_rd != uart1_tx_wr)) {
        UART1_TxStart();
    }

    __enable_irq();  // 退出临界区
}

/* ---------- 发送完成中断回调 ---------- */
void UART1_TX_DMACallback(void)
{
    uart1_tx_rd = (uart1_tx_rd + uart1_tx_len) % UART_TX_BUF_SIZE;
    uart1_tx_busy = 0;

    /* 如果队列中还有数据未发完，继续发送 */
    if (uart1_tx_rd != uart1_tx_wr) {
        UART1_TxStart();
    }
}

/* ---------- 接收空闲中断回调 ---------- */
void UART1_RX_IdleCallback(uint16_t len)
{
    uint16_t i;
    for (i = 0; i < len; i++) {
        uint8_t ch = uart1_dma_rx_buf[i];
        uint16_t next_head = (uart1_rx_buf.head + 1) % UART_RX_BUF_SIZE;

        if (next_head != uart1_rx_buf.tail) {   /* 队列未满 */
            uart1_rx_buf.buffer[uart1_rx_buf.head] = ch;
            uart1_rx_buf.head = next_head;
            uart1_rx_buf.count++;
        } else {
            /* 队列满，丢弃 */
        }
    }
}

/* ---------- 从接收队列取一个字符 ---------- */
uint8_t UART1_GetChar(uint8_t *ch)
{
    uint8_t ret = 0;
    if (ch == 0) {
        return 0;
    }

    __disable_irq();
    if (uart1_rx_buf.count > 0) {
        *ch = uart1_rx_buf.buffer[uart1_rx_buf.tail];
        uart1_rx_buf.tail = (uart1_rx_buf.tail + 1) % UART_RX_BUF_SIZE;
        uart1_rx_buf.count--;
        ret = 1;
    }
    __enable_irq();

    return ret;
}

/* ---------- 获取接收队列可用字节数 ---------- */
uint16_t UART1_Available(void)
{
    uint16_t cnt;
    __disable_irq();
    cnt = uart1_rx_buf.count;
    __enable_irq();
    return cnt;
}



/* ==================== UART2（K210通信）==================== */

static uint8_t uart2_tx_buf[UART_TX_BUF_SIZE];
static volatile uint16_t uart2_tx_wr = 0;
static volatile uint16_t uart2_tx_rd = 0;
static volatile uint8_t  uart2_tx_busy = 0;
static volatile uint16_t uart2_tx_len = 0;   // 当前DMA发送长度，用于回调更新读指针

static ring_buffer_t uart2_rx_buf = {{0}, 0, 0, 0};

uint8_t uart2_dma_rx_buf[UART_RX_BUF_SIZE];  // DMA接收缓冲区

/* 新增：UART2 DMA 接收当前位置（不改原有变量名，只增加一个游标） */
volatile uint16_t uart2_rx_pos = 0;

static void UART2_TxStart(void);

/* ---------- UART2 初始化 ---------- */
void UART2_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef  NVIC_InitStructure;
    DMA_InitTypeDef   DMA_InitStructure;

    /* 1) 时钟使能 */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

    /* 2) GPIO PD5(TX) PD6(RX) */
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_5 | GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    /* 3) USART2 配置：115200 8N1，无流控 */
    USART_InitStructure.USART_BaudRate            = 115200;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &USART_InitStructure);

    /* 4) 使能空闲中断 */
    USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);

    /* 5) DMA 接收配置 (DMA1 Stream5 Channel4) */
    DMA_DeInit(DMA1_Stream5);
    DMA_InitStructure.DMA_Channel            = DMA_Channel_4;
    DMA_InitStructure.DMA_PeripheralBaseAddr  = (uint32_t)&USART2->DR;
    DMA_InitStructure.DMA_Memory0BaseAddr     = (uint32_t)uart2_dma_rx_buf;
    DMA_InitStructure.DMA_DIR                 = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize          = UART_RX_BUF_SIZE;
    DMA_InitStructure.DMA_PeripheralInc       = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc           = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize  = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize      = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode                = DMA_Mode_Circular;   /* 关键优化：改为循环模式 */
    DMA_InitStructure.DMA_Priority            = DMA_Priority_High;
    DMA_InitStructure.DMA_FIFOMode            = DMA_FIFOMode_Disable;
    DMA_Init(DMA1_Stream5, &DMA_InitStructure);

    /* 6) DMA 发送配置 (DMA1 Stream6 Channel4) */
    DMA_DeInit(DMA1_Stream6);
    DMA_InitStructure.DMA_Channel            = DMA_Channel_4;
    DMA_InitStructure.DMA_PeripheralBaseAddr  = (uint32_t)&USART2->DR;
    DMA_InitStructure.DMA_Memory0BaseAddr     = (uint32_t)uart2_tx_buf;
    DMA_InitStructure.DMA_DIR                 = DMA_DIR_MemoryToPeripheral;
    DMA_InitStructure.DMA_BufferSize          = UART_TX_BUF_SIZE;
    DMA_InitStructure.DMA_PeripheralInc       = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc           = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize  = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize      = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode                = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority            = DMA_Priority_High;
    DMA_InitStructure.DMA_FIFOMode            = DMA_FIFOMode_Disable;
    DMA_Init(DMA1_Stream6, &DMA_InitStructure);

    /* 7) 配置 NVIC：USART2 和 DMA1_Stream6 中断 */
    NVIC_InitStructure.NVIC_IRQChannel                   = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream6_IRQn;
    NVIC_Init(&NVIC_InitStructure);

    /* 8) 使能 DMA 发送完成中断 */
    DMA_ITConfig(DMA1_Stream6, DMA_IT_TC, ENABLE);

    /* 9) 使能 DMA 请求 */
    USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);
    USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);

    /* 10) 启动 DMA 接收 */
    uart2_rx_pos = 0;
    DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF5 | DMA_FLAG_HTIF5 | DMA_FLAG_TEIF5 | DMA_FLAG_DMEIF5 | DMA_FLAG_FEIF5);
    DMA_Cmd(DMA1_Stream5, ENABLE);

    /* 11) 使能 USART2 */
    USART_Cmd(USART2, ENABLE);

    /* 12) 读一次 SR/DR 清除可能的空闲标志，避免上电后误进中断 */
    {
        uint32_t tmp;
        tmp = USART2->SR;
        tmp = USART2->DR;
        (void)tmp;
    }
}

/* ---------- 启动一次 TX DMA ---------- */
static void UART2_TxStart(void)
{
    uint16_t cnt = 0;
    uint16_t idx = uart2_tx_rd;

    if (uart2_tx_rd == uart2_tx_wr) {
        uart2_tx_busy = 0;
        return;
    }

    while (idx != uart2_tx_wr && cnt < UART_TX_BUF_SIZE) {
        cnt++;
        idx = (idx + 1) % UART_TX_BUF_SIZE;
    }

    if (cnt == 0) {
        uart2_tx_busy = 0;
        return;
    }

    /* 如果环形缓冲区跨边界，只发连续的第一段 */
    if ((uart2_tx_rd + cnt) > UART_TX_BUF_SIZE) {
        cnt = UART_TX_BUF_SIZE - uart2_tx_rd;
    }

    uart2_tx_len = cnt;

    /* 先关闭 DMA，再重设地址和长度 */
    DMA_Cmd(DMA1_Stream6, DISABLE);
    while (DMA_GetCmdStatus(DMA1_Stream6) != DISABLE) {
        ;
    }

    DMA_ClearFlag(DMA1_Stream6, DMA_FLAG_TCIF6 | DMA_FLAG_HTIF6 | DMA_FLAG_TEIF6 | DMA_FLAG_DMEIF6 | DMA_FLAG_FEIF6);

    DMA1_Stream6->M0AR = (uint32_t)&uart2_tx_buf[uart2_tx_rd];
    DMA1_Stream6->NDTR = cnt;
    DMA_Cmd(DMA1_Stream6, ENABLE);

    uart2_tx_busy = 1;
}

/* ---------- UART2 非阻塞发送 ---------- */
void UART2_SendData_NonBlocking(uint8_t *data, uint16_t len)
{
    uint16_t i;

    if (data == 0 || len == 0) {
        return;
    }

    __disable_irq();  // 进入临界区

    for (i = 0; i < len; i++) {
        uint16_t next_wr = (uart2_tx_wr + 1) % UART_TX_BUF_SIZE;
        if (next_wr != uart2_tx_rd) {  // 队列未满
            uart2_tx_buf[uart2_tx_wr] = data[i];
            uart2_tx_wr = next_wr;
        } else {
            /* 队列满则丢弃后续数据 */
            break;
        }
    }

    /* 如果发送空闲，立即启动一次发送 */
    if (!uart2_tx_busy && (uart2_tx_rd != uart2_tx_wr)) {
        UART2_TxStart();
    }

    __enable_irq();  // 退出临界区
}

/* ---------- UART2 发送完成中断回调 ---------- */
void UART2_TX_DMACallback(void)
{
    uart2_tx_rd = (uart2_tx_rd + uart2_tx_len) % UART_TX_BUF_SIZE;
    uart2_tx_busy = 0;

    /* 如果队列中还有数据未发完，继续发送 */
    if (uart2_tx_rd != uart2_tx_wr) {
        UART2_TxStart();
    }
}

/* ---------- UART2 接收空闲中断回调 ---------- */
void UART2_RX_IdleCallback(uint16_t len)
{
    uint16_t i;
    uint16_t pos;

    pos = uart2_rx_pos;

    for (i = 0; i < len; i++) {
        uint8_t ch = uart2_dma_rx_buf[pos];
        uint16_t next_head = (uart2_rx_buf.head + 1) % UART_RX_BUF_SIZE;

        pos++;
        if (pos >= UART_RX_BUF_SIZE) {
            pos = 0;
        }

        if (next_head != uart2_rx_buf.tail) {   /* 队列未满 */
            uart2_rx_buf.buffer[uart2_rx_buf.head] = ch;
            uart2_rx_buf.head = next_head;
            uart2_rx_buf.count++;
        } else {
            /* 队列满，丢弃 */
        }
    }

    uart2_rx_pos = pos;
}

/* ---------- 从 UART2 接收队列取一个字符 ---------- */
uint8_t UART2_GetChar(uint8_t *ch)
{
    uint8_t ret = 0;

    if (ch == 0) {
        return 0;
    }

    __disable_irq();

    if (uart2_rx_buf.count > 0) {
        *ch = uart2_rx_buf.buffer[uart2_rx_buf.tail];
        uart2_rx_buf.tail = (uart2_rx_buf.tail + 1) % UART_RX_BUF_SIZE;
        uart2_rx_buf.count--;
        ret = 1;
    }

    __enable_irq();

    return ret;
}

/* ---------- 获取 UART2 接收队列可用字节数 ---------- */
uint16_t UART2_Available(void)
{
    uint16_t cnt;

    __disable_irq();
    cnt = uart2_rx_buf.count;
    __enable_irq();

    return cnt;
}
