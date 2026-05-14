#ifndef __BSP_SPI_H
#define __BSP_SPI_H

#include "stm32f4xx.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * SPI1:
 *   PA5 -> SCK
 *   PA6 -> MISO
 *   PA7 -> MOSI
 *
 * PMW3901 使用 SPI Mode 3:
 *   CPOL = 1
 *   CPHA = 1
 */
typedef void (*spi1_done_cb_t)(void *ctx, uint8_t status);

/*
 * status:
 *   0 = OK
 *   1 = DMA/SPI error
 *   2 = parameter error
 *   3 = busy
 *   4 = timeout
 */
#define SPI1_STATUS_OK       0U
#define SPI1_STATUS_ERROR    1U
#define SPI1_STATUS_PARAM    2U
#define SPI1_STATUS_BUSY     3U
#define SPI1_STATUS_TIMEOUT  4U

void SPI1_Init(void);

/* 阻塞收发 1 字节：成功返回 1，失败返回 0 */
uint8_t SPI1_TransferByte(uint8_t tx, uint8_t *rx);

/* 兼容旧接口：失败时返回 0xFF */
uint8_t SPI1_ReadWriteByte(uint8_t data);

/*
 * DMA 非阻塞收发：
 * 返回值：
 *   1 = 成功启动，完成后进入 cb
 *   0 = 未启动
 *
 * 注意：
 *   - tx_buf/rx_buf 必须在 DMA 完成前保持有效
 *   - cb 在 DMA 中断上下文中调用，只能置标志，不要 printf
 */
uint8_t SPI1_TransferAsync_DMA(uint8_t *tx_buf,
                               uint8_t *rx_buf,
                               uint16_t len,
                               spi1_done_cb_t cb,
                               void *ctx);

uint8_t SPI1_IsBusy(void);

/* 给 stm32f4xx_it.c 调用 */
void SPI1_DMA_IRQHandler_RX(void);
void SPI1_DMA_IRQHandler_TX(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_SPI_H */
