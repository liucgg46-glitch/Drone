#include "bsp_i2c.h"
#include "bsp_timer.h"
#include "bsp_uart.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_dma.h"
#include "misc.h"
#include <stdio.h>
#include <string.h>

#define I2C_BLOCK_TIMEOUT          3000UL
#define I2C_ASYNC_TIMEOUT_MS       20U
#define I2C_DMA_TX_MAX_LEN         32U

#define I2C1_DMA_RX_STREAM         DMA1_Stream0
#define I2C1_DMA_TX_STREAM         DMA1_Stream7
#define I2C1_DMA_CHANNEL           DMA_Channel_1

/* ==================== DMA + 中断状态机 ==================== */
typedef enum {
    I2C_SM_IDLE = 0,
    I2C_SM_START_W,
    I2C_SM_ADDR_W,
    I2C_SM_REG_SENT,
    I2C_SM_TX_DMA,
    I2C_SM_TX_WAIT_BTF,
    I2C_SM_START_R,
    I2C_SM_ADDR_R,
    I2C_SM_RX_ONE,
    I2C_SM_RX_DMA
} I2C_State_t;

typedef struct {
    volatile I2C_State_t state;
    uint8_t  dev_addr;          /* 7-bit address: 0x68, 0x76... */
    uint8_t  reg_addr;
    uint8_t *buf;
    uint16_t len;
    uint8_t  is_read;
    uint32_t start_tick;
    I2C_Callback callback;
} I2C_Transaction_t;

static volatile I2C_Transaction_t g_i2c = { I2C_SM_IDLE, 0, 0, 0, 0, 0, 0, 0 };
static uint8_t g_i2c_tx_copy[I2C_DMA_TX_MAX_LEN];

/* ==================== 内部工具函数 ==================== */
static uint8_t I2C_WaitEvent(uint32_t event)
{
    uint32_t timeout = I2C_BLOCK_TIMEOUT;
    while (!I2C_CheckEvent(I2C1, event)) {
        if (--timeout == 0) return 0;
    }
    return 1;
}

static uint8_t DMA_WaitDisable(DMA_Stream_TypeDef *stream)
{
    uint32_t timeout = 10000;
    while (DMA_GetCmdStatus(stream) != DISABLE) {
        if (--timeout == 0) return 0;
    }
    return 1;
}

static void I2C1_DMA_ClearFlags(void)
{
    DMA_ClearFlag(I2C1_DMA_RX_STREAM,
                  DMA_FLAG_TCIF0 | DMA_FLAG_HTIF0 | DMA_FLAG_TEIF0 |
                  DMA_FLAG_DMEIF0 | DMA_FLAG_FEIF0);

    DMA_ClearFlag(I2C1_DMA_TX_STREAM,
                  DMA_FLAG_TCIF7 | DMA_FLAG_HTIF7 | DMA_FLAG_TEIF7 |
                  DMA_FLAG_DMEIF7 | DMA_FLAG_FEIF7);
}

static void I2C1_StopAndResetPeripheral(void)
{
    I2C_GenerateSTOP(I2C1, ENABLE);
    I2C_DMACmd(I2C1, DISABLE);
    I2C_DMALastTransferCmd(I2C1, DISABLE);
    I2C_AcknowledgeConfig(I2C1, ENABLE);

    I2C_ITConfig(I2C1, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR, DISABLE);

    DMA_Cmd(I2C1_DMA_RX_STREAM, DISABLE);
    DMA_Cmd(I2C1_DMA_TX_STREAM, DISABLE);
    (void)DMA_WaitDisable(I2C1_DMA_RX_STREAM);
    (void)DMA_WaitDisable(I2C1_DMA_TX_STREAM);
    I2C1_DMA_ClearFlags();

    I2C_ClearITPendingBit(I2C1, I2C_IT_BERR | I2C_IT_ARLO |
                                I2C_IT_AF   | I2C_IT_OVR);

    /* STM32F4 硬件 I2C 偶尔 BUSY 锁死，错误/超时时复位一下外设更稳 */
    I2C_SoftwareResetCmd(I2C1, ENABLE);
    for (volatile uint32_t i = 0; i < 1000; i++) { ; }
    I2C_SoftwareResetCmd(I2C1, DISABLE);
    I2C_Cmd(I2C1, ENABLE);
    I2C_AcknowledgeConfig(I2C1, ENABLE);
}

static void I2C1_Finish(int result)
{
    I2C_Callback cb = g_i2c.callback;

    I2C_ITConfig(I2C1, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR, DISABLE);
    I2C_DMACmd(I2C1, DISABLE);
    I2C_DMALastTransferCmd(I2C1, DISABLE);
    I2C_AcknowledgeConfig(I2C1, ENABLE);

    g_i2c.state = I2C_SM_IDLE;
    g_i2c.callback = 0;

    if (cb) {
        cb(result);
    }
}

static void I2C1_Abort(int result)
{
    I2C_Callback cb = g_i2c.callback;

    I2C1_StopAndResetPeripheral();
    g_i2c.state = I2C_SM_IDLE;
    g_i2c.callback = 0;

    if (cb) {
        cb(result);
    }
}

static uint8_t I2C1_ConfigTxDMA(uint8_t *buf, uint16_t len)
{
    DMA_InitTypeDef dma;

    DMA_Cmd(I2C1_DMA_TX_STREAM, DISABLE);
    if (!DMA_WaitDisable(I2C1_DMA_TX_STREAM)) return 0;

    DMA_DeInit(I2C1_DMA_TX_STREAM);
    DMA_StructInit(&dma);
    dma.DMA_Channel            = I2C1_DMA_CHANNEL;
    dma.DMA_PeripheralBaseAddr = (uint32_t)&I2C1->DR;
    dma.DMA_Memory0BaseAddr    = (uint32_t)buf;
    dma.DMA_DIR                = DMA_DIR_MemoryToPeripheral;
    dma.DMA_BufferSize         = len;
    dma.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dma.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    dma.DMA_Mode               = DMA_Mode_Normal;
    dma.DMA_Priority           = DMA_Priority_VeryHigh;
    dma.DMA_FIFOMode           = DMA_FIFOMode_Disable;
    DMA_Init(I2C1_DMA_TX_STREAM, &dma);

    DMA_ClearFlag(I2C1_DMA_TX_STREAM,
                  DMA_FLAG_TCIF7 | DMA_FLAG_HTIF7 | DMA_FLAG_TEIF7 |
                  DMA_FLAG_DMEIF7 | DMA_FLAG_FEIF7);
    DMA_ITConfig(I2C1_DMA_TX_STREAM, DMA_IT_TC | DMA_IT_TE | DMA_IT_DME | DMA_IT_FE, ENABLE);
    return 1;
}

static uint8_t I2C1_ConfigRxDMA(uint8_t *buf, uint16_t len)
{
    DMA_InitTypeDef dma;

    DMA_Cmd(I2C1_DMA_RX_STREAM, DISABLE);
    if (!DMA_WaitDisable(I2C1_DMA_RX_STREAM)) return 0;

    DMA_DeInit(I2C1_DMA_RX_STREAM);
    DMA_StructInit(&dma);
    dma.DMA_Channel            = I2C1_DMA_CHANNEL;
    dma.DMA_PeripheralBaseAddr = (uint32_t)&I2C1->DR;
    dma.DMA_Memory0BaseAddr    = (uint32_t)buf;
    dma.DMA_DIR                = DMA_DIR_PeripheralToMemory;
    dma.DMA_BufferSize         = len;
    dma.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dma.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    dma.DMA_Mode               = DMA_Mode_Normal;
    dma.DMA_Priority           = DMA_Priority_VeryHigh;
    dma.DMA_FIFOMode           = DMA_FIFOMode_Disable;
    DMA_Init(I2C1_DMA_RX_STREAM, &dma);

    DMA_ClearFlag(I2C1_DMA_RX_STREAM,
                  DMA_FLAG_TCIF0 | DMA_FLAG_HTIF0 | DMA_FLAG_TEIF0 |
                  DMA_FLAG_DMEIF0 | DMA_FLAG_FEIF0);
    DMA_ITConfig(I2C1_DMA_RX_STREAM, DMA_IT_TC | DMA_IT_TE | DMA_IT_DME | DMA_IT_FE, ENABLE);
    return 1;
}

static uint8_t I2C1_BeginAsync(uint8_t dev_addr, uint8_t reg_addr,
                               uint8_t *buf, uint16_t len,
                               uint8_t is_read, I2C_Callback callback)
{
    uint32_t primask;

    if (buf == 0 || len == 0) return 2;

    primask = __get_PRIMASK();
    __disable_irq();

    if (g_i2c.state != I2C_SM_IDLE) {
        if (!primask) __enable_irq();
        return 1;
    }

    if ((I2C1->SR2 & I2C_SR2_BUSY) != 0) {
        if (!primask) __enable_irq();
        return 2;
    }

    g_i2c.dev_addr   = dev_addr;
    g_i2c.reg_addr   = reg_addr;
    g_i2c.buf        = buf;
    g_i2c.len        = len;
    g_i2c.is_read    = is_read;
    g_i2c.callback   = callback;
    g_i2c.start_tick = GetTick();
    g_i2c.state      = I2C_SM_START_W;

    I2C_AcknowledgeConfig(I2C1, ENABLE);
    I2C_ITConfig(I2C1, I2C_IT_ERR, ENABLE);
    I2C_ITConfig(I2C1, I2C_IT_EVT, ENABLE);
    I2C_GenerateSTART(I2C1, ENABLE);

    if (!primask) __enable_irq();
    return 0;
}

/* ==================== 初始化 ==================== */
void I2C1_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    I2C_InitTypeDef  I2C_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_I2C1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    I2C_DeInit(I2C1);
    I2C_InitStructure.I2C_ClockSpeed           = 100000;  /* 先 100kHz，稳定后再改 400kHz */
    I2C_InitStructure.I2C_Mode                 = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle            = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1          = 0x00;
    I2C_InitStructure.I2C_Ack                  = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress  = I2C_AcknowledgedAddress_7bit;
    I2C_Init(I2C1, &I2C_InitStructure);

    I2C_Cmd(I2C1, ENABLE);
    I2C_AcknowledgeConfig(I2C1, ENABLE);

    /* 先关 DMA，真正传输时再配置并打开 */
    DMA_Cmd(I2C1_DMA_RX_STREAM, DISABLE);
    DMA_Cmd(I2C1_DMA_TX_STREAM, DISABLE);
    I2C1_DMA_ClearFlags();

    /* I2C 事件/错误中断 */
    NVIC_InitStructure.NVIC_IRQChannel = I2C1_EV_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = I2C1_ER_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&NVIC_InitStructure);

    /* I2C1_RX = DMA1 Stream0 Channel1 */
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_Init(&NVIC_InitStructure);

    /* I2C1_TX = DMA1 Stream7 Channel1，避开你的 USART2_TX DMA1 Stream6 */
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream7_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&NVIC_InitStructure);

    I2C_ITConfig(I2C1, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR, DISABLE);
    g_i2c.state = I2C_SM_IDLE;

    for (volatile uint32_t i = 0; i < 10000; i++) { ; }
}

/* ==================== 保留的阻塞 API ==================== */
static void I2C_ClearErrorAndStop_Blocking(void)
{
    I2C_GenerateSTOP(I2C1, ENABLE);
    I2C_ClearITPendingBit(I2C1, I2C_IT_BERR | I2C_IT_ARLO | I2C_IT_AF | I2C_IT_OVR);
    I2C_AcknowledgeConfig(I2C1, ENABLE);
}

uint8_t I2C1_WriteByte(uint8_t dev_addr, uint8_t reg_addr, uint8_t data)
{
    return I2C1_WriteBytes(dev_addr, reg_addr, &data, 1);
}

uint8_t I2C1_WriteBytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    if (I2C1_IsBusy()) return 2;

    I2C_GenerateSTART(I2C1, ENABLE);
    if (!I2C_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT)) goto error;

    I2C_Send7bitAddress(I2C1, dev_addr << 1, I2C_Direction_Transmitter);
    if (!I2C_WaitEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) goto error;

    I2C_SendData(I2C1, reg_addr);
    if (!I2C_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) goto error;

    for (uint16_t i = 0; i < len; i++) {
        I2C_SendData(I2C1, data[i]);
        if (!I2C_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) goto error;
    }

    I2C_GenerateSTOP(I2C1, ENABLE);
    return 0;

error:
    I2C_ClearErrorAndStop_Blocking();
    return 1;
}

uint8_t I2C1_ReadByte(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data)
{
    return I2C1_ReadBytes(dev_addr, reg_addr, data, 1);
}

uint8_t I2C1_ReadBytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t *buf, uint16_t len)
{
    if (len == 0) return 0;
    if (I2C1_IsBusy()) return 2;

    I2C_GenerateSTART(I2C1, ENABLE);
    if (!I2C_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT)) goto error;

    I2C_Send7bitAddress(I2C1, dev_addr << 1, I2C_Direction_Transmitter);
    if (!I2C_WaitEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) goto error;

    I2C_SendData(I2C1, reg_addr);
    if (!I2C_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) goto error;

    I2C_GenerateSTART(I2C1, ENABLE);
    if (!I2C_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT)) goto error;

    I2C_Send7bitAddress(I2C1, dev_addr << 1, I2C_Direction_Receiver);
    if (!I2C_WaitEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) goto error;

    for (uint16_t i = 0; i < len; i++) {
        if (i == len - 1) {
            I2C_AcknowledgeConfig(I2C1, DISABLE);
        }
        if (!I2C_WaitEvent(I2C_EVENT_MASTER_BYTE_RECEIVED)) goto error;
        buf[i] = I2C_ReceiveData(I2C1);
    }

    I2C_GenerateSTOP(I2C1, ENABLE);
    I2C_AcknowledgeConfig(I2C1, ENABLE);
    return 0;

error:
    I2C_ClearErrorAndStop_Blocking();
    I2C_AcknowledgeConfig(I2C1, ENABLE);
    return 1;
}

void I2C1_ScanDevices(void)
{
    uint8_t device_addr;
    char buf[64];

    if (I2C1_IsBusy()) {
        UART1_SendData_NonBlocking((uint8_t*)"I2C async busy, scan skipped.\r\n", 31);
        return;
    }

    for (device_addr = 0x08; device_addr <= 0x77; device_addr++) {
        uint32_t timeout;

        timeout = 10000;
        while (I2C1->SR2 & I2C_SR2_BUSY) {
            if (--timeout == 0) goto next;
        }

        I2C_GenerateSTART(I2C1, ENABLE);
        timeout = 10000;
        while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)) {
            if (--timeout == 0) goto stop_and_next;
        }

        I2C_Send7bitAddress(I2C1, device_addr << 1, I2C_Direction_Transmitter);
        timeout = 10000;
        while (1) {
            uint32_t sr1 = I2C1->SR1;

            if (sr1 & I2C_SR1_ADDR) {
                volatile uint32_t temp;
                temp = I2C1->SR1;
                temp = I2C1->SR2;
                (void)temp;
                snprintf(buf, sizeof(buf), "Found I2C device: 0x%02X\r\n", device_addr);
                UART1_SendData_NonBlocking((uint8_t*)buf, strlen(buf));
                break;
            }

            if (sr1 & I2C_SR1_AF) {
                I2C_ClearFlag(I2C1, I2C_FLAG_AF);
                break;
            }

            if (--timeout == 0) break;
        }

stop_and_next:
        I2C_GenerateSTOP(I2C1, ENABLE);
        timeout = 10000;
        while (I2C1->SR2 & I2C_SR2_BUSY) {
            if (--timeout == 0) break;
        }
next:
        ;
    }

    UART1_SendData_NonBlocking((uint8_t*)"I2C scan finished.\r\n", 20);
}

/* ==================== 非阻塞 API ==================== */
uint8_t I2C1_WriteBytes_DMA_Async(uint8_t dev_addr, uint8_t reg_addr,
                                  const uint8_t *tx_data, uint16_t len,
                                  I2C_Callback callback)
{
    if (tx_data == 0 || len == 0) return 2;
    if (len > I2C_DMA_TX_MAX_LEN) return 2;
    if (I2C1_IsBusy()) return 1;

    memcpy(g_i2c_tx_copy, tx_data, len);
    return I2C1_BeginAsync(dev_addr, reg_addr, g_i2c_tx_copy, len, 0, callback);
}

uint8_t I2C1_ReadBytes_DMA_Async(uint8_t dev_addr, uint8_t reg_addr,
                                 uint8_t *rx_buf, uint16_t len,
                                 I2C_Callback callback)
{
    return I2C1_BeginAsync(dev_addr, reg_addr, rx_buf, len, 1, callback);
}

uint8_t I2C1_IsBusy(void)
{
    return (g_i2c.state != I2C_SM_IDLE);
}

void I2C1_Task(void)
{
    if (g_i2c.state != I2C_SM_IDLE) {
        if ((GetTick() - g_i2c.start_tick) > I2C_ASYNC_TIMEOUT_MS) {
            I2C1_Abort(-2);
        }
    }
}

/* ==================== ISR 状态机 ==================== */
void I2C1_EV_ISR(void)
{
    uint32_t sr1 = I2C1->SR1;

    switch (g_i2c.state) {
    case I2C_SM_START_W:
        if (sr1 & I2C_SR1_SB) {
            I2C_Send7bitAddress(I2C1, g_i2c.dev_addr << 1, I2C_Direction_Transmitter);
            g_i2c.state = I2C_SM_ADDR_W;
        }
        break;

    case I2C_SM_ADDR_W:
        if (sr1 & I2C_SR1_ADDR) {
            (void)I2C1->SR1;
            (void)I2C1->SR2;
            I2C_SendData(I2C1, g_i2c.reg_addr);
            g_i2c.state = I2C_SM_REG_SENT;
        }
        break;

    case I2C_SM_REG_SENT:
        if (sr1 & I2C_SR1_BTF) {
            if (g_i2c.is_read) {
                I2C_GenerateSTART(I2C1, ENABLE);
                g_i2c.state = I2C_SM_START_R;
            } else {
                if (!I2C1_ConfigTxDMA(g_i2c.buf, g_i2c.len)) {
                    I2C1_Abort(-1);
                    return;
                }

                g_i2c.state = I2C_SM_TX_DMA;
                I2C_ITConfig(I2C1, I2C_IT_EVT | I2C_IT_BUF, DISABLE);
                I2C_DMACmd(I2C1, ENABLE);
                DMA_Cmd(I2C1_DMA_TX_STREAM, ENABLE);
            }
        }
        break;

    case I2C_SM_START_R:
        if (sr1 & I2C_SR1_SB) {
            I2C_Send7bitAddress(I2C1, g_i2c.dev_addr << 1, I2C_Direction_Receiver);
            g_i2c.state = I2C_SM_ADDR_R;
        }
        break;

    case I2C_SM_ADDR_R:
        if (sr1 & I2C_SR1_ADDR) {
            if (g_i2c.len == 1) {
                /* 单字节读：ADDR 清除前关闭 ACK，ADDR 清除后立即 STOP，再等 RXNE 读 DR */
                I2C_AcknowledgeConfig(I2C1, DISABLE);
                (void)I2C1->SR1;
                (void)I2C1->SR2;
                I2C_GenerateSTOP(I2C1, ENABLE);
                g_i2c.state = I2C_SM_RX_ONE;
                I2C_ITConfig(I2C1, I2C_IT_BUF, ENABLE);
            } else {
                if (!I2C1_ConfigRxDMA(g_i2c.buf, g_i2c.len)) {
                    I2C1_Abort(-1);
                    return;
                }

                g_i2c.state = I2C_SM_RX_DMA;
                I2C_DMALastTransferCmd(I2C1, ENABLE);
                I2C_DMACmd(I2C1, ENABLE);
                DMA_Cmd(I2C1_DMA_RX_STREAM, ENABLE);

                /* 清 ADDR 后从机才会真正开始送数据，此时 DMA 已经准备好 */
                (void)I2C1->SR1;
                (void)I2C1->SR2;
                I2C_ITConfig(I2C1, I2C_IT_EVT | I2C_IT_BUF, DISABLE);
            }
        }
        break;

    case I2C_SM_RX_ONE:
        if (sr1 & I2C_SR1_RXNE) {
            g_i2c.buf[0] = I2C_ReceiveData(I2C1);
            I2C1_Finish(0);
        }
        break;

    case I2C_SM_TX_WAIT_BTF:
        if (sr1 & I2C_SR1_BTF) {
            I2C_GenerateSTOP(I2C1, ENABLE);
            I2C1_Finish(0);
        }
        break;

    default:
        break;
    }
}

void I2C1_ER_ISR(void)
{
    uint32_t sr1 = I2C1->SR1;

    if (sr1 & (I2C_SR1_BERR | I2C_SR1_ARLO | I2C_SR1_AF | I2C_SR1_OVR)) {
        I2C_ClearITPendingBit(I2C1, I2C_IT_BERR | I2C_IT_ARLO |
                                    I2C_IT_AF   | I2C_IT_OVR);
        I2C1_Abort(-1);
    }
}

void I2C1_DMA_RX_ISR(void)
{
    if (DMA_GetITStatus(I2C1_DMA_RX_STREAM, DMA_IT_TEIF0) != RESET ||
        DMA_GetITStatus(I2C1_DMA_RX_STREAM, DMA_IT_DMEIF0) != RESET ||
        DMA_GetITStatus(I2C1_DMA_RX_STREAM, DMA_IT_FEIF0) != RESET) {

        DMA_ClearITPendingBit(I2C1_DMA_RX_STREAM,
                              DMA_IT_TEIF0 | DMA_IT_DMEIF0 | DMA_IT_FEIF0);
        I2C1_Abort(-1);
        return;
    }

    if (DMA_GetITStatus(I2C1_DMA_RX_STREAM, DMA_IT_TCIF0) != RESET) {
        DMA_ClearITPendingBit(I2C1_DMA_RX_STREAM, DMA_IT_TCIF0);
        DMA_Cmd(I2C1_DMA_RX_STREAM, DISABLE);
        I2C_DMACmd(I2C1, DISABLE);
        I2C_DMALastTransferCmd(I2C1, DISABLE);
        I2C_GenerateSTOP(I2C1, ENABLE);
        I2C1_Finish(0);
    }
}

void I2C1_DMA_TX_ISR(void)
{
    if (DMA_GetITStatus(I2C1_DMA_TX_STREAM, DMA_IT_TEIF7) != RESET ||
        DMA_GetITStatus(I2C1_DMA_TX_STREAM, DMA_IT_DMEIF7) != RESET ||
        DMA_GetITStatus(I2C1_DMA_TX_STREAM, DMA_IT_FEIF7) != RESET) {

        DMA_ClearITPendingBit(I2C1_DMA_TX_STREAM,
                              DMA_IT_TEIF7 | DMA_IT_DMEIF7 | DMA_IT_FEIF7);
        I2C1_Abort(-1);
        return;
    }

    if (DMA_GetITStatus(I2C1_DMA_TX_STREAM, DMA_IT_TCIF7) != RESET) {
        DMA_ClearITPendingBit(I2C1_DMA_TX_STREAM, DMA_IT_TCIF7);
        DMA_Cmd(I2C1_DMA_TX_STREAM, DISABLE);
        I2C_DMACmd(I2C1, DISABLE);

        /* DMA 完成只代表数据进了 I2C->DR，不代表最后一个字节已经上总线。
         * 这里转到 BTF 状态，由 I2C 事件中断确认真正发送完成后再 STOP。
         */
        g_i2c.state = I2C_SM_TX_WAIT_BTF;
        I2C_ITConfig(I2C1, I2C_IT_EVT, ENABLE);
    }
}
