#ifndef __BSP_I2C_H
#define __BSP_I2C_H

#include "stm32f4xx.h"
#include <stdint.h>

/*
 * result:
 *   0  = success
 *  -1  = I2C/DMA error, NACK, arbitration lost, bus error, etc.
 *  -2  = software timeout
 */
typedef void (*I2C_Callback)(int result);

/* 阻塞 API：保留给上电初始化、单次扫描、临时调试用 */
void I2C1_Init(void);
uint8_t I2C1_WriteByte(uint8_t dev_addr, uint8_t reg_addr, uint8_t data);
uint8_t I2C1_WriteBytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);
uint8_t I2C1_ReadByte(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data);
uint8_t I2C1_ReadBytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t *buf, uint16_t len);
void I2C1_ScanDevices(void);

/*
 * 非阻塞 DMA + 状态机 API
 * 返回值：
 *   0 = 已经成功启动，完成后进 callback
 *   1 = I2C 状态机忙，本次没有启动
 *   2 = 参数错误 / 总线忙，本次没有启动
 *
 * 注意：
 *   - Read 的 rx_buf 必须保持有效，直到 callback 被调用。
 *   - Write 的 tx_data 会被复制到驱动内部缓冲区，调用后原数组可以释放。
 */
uint8_t I2C1_WriteBytes_DMA_Async(uint8_t dev_addr, uint8_t reg_addr,
                                  const uint8_t *tx_data, uint16_t len,
                                  I2C_Callback callback);
uint8_t I2C1_ReadBytes_DMA_Async(uint8_t dev_addr, uint8_t reg_addr,
                                 uint8_t *rx_buf, uint16_t len,
                                 I2C_Callback callback);

uint8_t I2C1_IsBusy(void);
void I2C1_Task(void);      /* 建议 1ms~5ms 调一次，用来处理超时保护 */

/* 放在 stm32f4xx_it.c 的 IRQHandler 里调用 */
void I2C1_EV_ISR(void);
void I2C1_ER_ISR(void);
void I2C1_DMA_RX_ISR(void);    /* DMA1_Stream0_IRQHandler */
void I2C1_DMA_TX_ISR(void);    /* DMA1_Stream7_IRQHandler */

#endif
