#ifndef __BSP_UART_H
#define __BSP_UART_H

#include <stdint.h>

/* 环形队列大小（可根据内存调整） */
#define UART_RX_BUF_SIZE    256
#define UART_TX_BUF_SIZE    512

/* 环形队列结构体 */
typedef struct {
    uint8_t  buffer[UART_RX_BUF_SIZE];
    uint16_t head;          // 写入位置
    uint16_t tail;          // 读取位置
    uint16_t count;         // 已存储字节数
} ring_buffer_t;

//------------------USART1----------------------
extern uint8_t uart1_dma_rx_buf[UART_RX_BUF_SIZE];

/* 串口初始化 */
void UART1_Init(void);      // 调试口 PA9(TX) PA10(RX)

/* 接收相关的函数 */
uint8_t UART1_GetChar(uint8_t *ch);     // 从接收队列读一个字节，成功返回1
uint16_t UART1_Available(void);         // 返回队列中可读字节数

/* 非阻塞发送 */
void UART1_SendData_NonBlocking(uint8_t *data, uint16_t len);

/* 供中断服务函数调用的接口 */
void UART1_RX_IdleCallback(uint16_t len);
void UART1_TX_DMACallback(void);

//------------------USART2----------------------
extern uint8_t uart2_dma_rx_buf[UART_RX_BUF_SIZE];
extern volatile uint16_t uart2_rx_pos;
void UART2_Init(void);

uint8_t UART2_GetChar(uint8_t *ch);
uint16_t UART2_Available(void);

void UART2_SendData_NonBlocking(uint8_t *data, uint16_t len);

void UART2_RX_IdleCallback(uint16_t len);
void UART2_TX_DMACallback(void);

#endif
