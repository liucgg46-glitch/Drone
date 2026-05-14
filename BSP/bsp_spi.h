#ifndef __BSP_SPI_H
#define __BSP_SPI_H

#include "stm32f4xx.h"
#include <stdint.h>

typedef void (*spi1_done_cb_t)(void *ctx, uint8_t status);

void SPI1_Init(void);

/* 保留原来的阻塞接口 */
uint8_t SPI1_ReadWriteByte(uint8_t data);

/* 新增 DMA 无阻塞接口 */
uint8_t SPI1_TransferAsync_DMA(uint8_t *tx_buf,
                               uint8_t *rx_buf,
                               uint16_t len,
                               spi1_done_cb_t cb,
                               void *ctx);

uint8_t SPI1_IsBusy(void);

/* 给 stm32f4xx_it.c 调用 */
void SPI1_DMA_IRQHandler_RX(void);
void SPI1_DMA_IRQHandler_TX(void);

#endif
