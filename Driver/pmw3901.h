#ifndef __PMW3901_H
#define __PMW3901_H

#include "stm32f4xx.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==================== PMW3901 寄存器 ==================== */
#define PMW3901_REG_PRODUCT_ID      0x00U
#define PMW3901_REG_REVISION_ID     0x01U
#define PMW3901_REG_MOTION          0x02U
#define PMW3901_REG_DELTA_X_L       0x03U
#define PMW3901_REG_DELTA_X_H       0x04U
#define PMW3901_REG_DELTA_Y_L       0x05U
#define PMW3901_REG_DELTA_Y_H       0x06U
#define PMW3901_REG_SQUAL           0x07U
#define PMW3901_REG_MOTION_BURST    0x16U
#define PMW3901_REG_POWER_UP_RESET  0x3AU

#define PMW3901_PRODUCT_ID_VALUE    0x49U

/* 兼容你给的旧名字 */
#define PMW3901_PRODUCT_ID          PMW3901_REG_PRODUCT_ID
#define PMW3901_REVISION_ID         PMW3901_REG_REVISION_ID
#define PMW3901_MOTION              PMW3901_REG_MOTION
#define PMW3901_DELTA_X_L           PMW3901_REG_DELTA_X_L
#define PMW3901_DELTA_X_H           PMW3901_REG_DELTA_X_H
#define PMW3901_DELTA_Y_L           PMW3901_REG_DELTA_Y_L
#define PMW3901_DELTA_Y_H           PMW3901_REG_DELTA_Y_H
#define PMW3901_MOTION_BURST        PMW3901_REG_MOTION_BURST

/* ==================== 引脚配置 ====================
 * 默认按你发来的设计：
 *   PB0 -> CS/NCS
 *   PB1 -> RST
 *
 * SPI1:
 *   PA5 -> SCK
 *   PA6 -> MISO
 *   PA7 -> MOSI
 */
#define PMW3901_CS_PORT             GPIOB
#define PMW3901_CS_PIN              GPIO_Pin_0
#define PMW3901_RST_PORT            GPIOB
#define PMW3901_RST_PIN             GPIO_Pin_1
#define PMW3901_GPIO_RCC            RCC_AHB1Periph_GPIOB

/* ==================== 返回值 ==================== */
#define PMW3901_OK                  1U
#define PMW3901_FAIL                0U

typedef struct
{
    int16_t dx;
    int16_t dy;

    uint8_t motion;
    uint8_t quality;        /* SQUAL */

    uint8_t motion_detected;/* motion bit */
    uint8_t overflow;       /* overflow bit */
    uint16_t shutter;

    uint32_t stamp_ms;
    uint8_t valid;
} pmw3901_t;

void PMW3901_GPIO_Init(void);
uint8_t PMW3901_Init(void);

uint8_t PMW3901_ReadReg(uint8_t reg);
void PMW3901_WriteReg(uint8_t reg, uint8_t data);

uint8_t PMW3901_ReadID(void);
uint8_t PMW3901_ReadRevisionID(void);

/* 阻塞读取：用于先确认光流是否正常 */
uint8_t PMW3901_ReadMotion(pmw3901_t *flow);
uint8_t PMW3901_ReadMotionBurst_Blocking(pmw3901_t *flow, uint8_t raw[12]);

/* 异步 DMA motion burst：运行阶段用 */
uint8_t PMW3901_ReadMotion_Async(pmw3901_t *flow);

/* 放在 while(1) 或 1ms 任务里一直调用 */
void PMW3901_Task(void);

/* 判断异步读取是否完成；读到 1 后本次 flow 数据已更新 */
uint8_t PMW3901_ReadMotion_IsDone(void);

/* 判断 PMW3901 状态机是否忙 */
uint8_t PMW3901_IsBusy(void);

/* 调试：拷贝最近一次 async burst 原始 12 字节 */
void PMW3901_GetAsyncRaw(uint8_t *buf);

#ifdef __cplusplus
}
#endif

#endif /* __PMW3901_H */
