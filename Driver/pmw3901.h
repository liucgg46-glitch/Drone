#ifndef __PMW3901_H
#define __PMW3901_H

#include "stm32f4xx.h"
#include <stdint.h>

#define PMW3901_PRODUCT_ID      0x00
#define PMW3901_REVISION_ID     0x01
#define PMW3901_MOTION          0x02
#define PMW3901_DELTA_X_L       0x03
#define PMW3901_DELTA_X_H       0x04
#define PMW3901_DELTA_Y_L       0x05
#define PMW3901_DELTA_Y_H       0x06
#define PMW3901_MOTION_BURST    0x16
typedef struct
{
    int16_t dx;
    int16_t dy;
    uint8_t motion;
    uint8_t quality;
} pmw3901_t;

void PMW3901_GPIO_Init(void);
uint8_t PMW3901_Init(void);

uint8_t PMW3901_ReadReg(uint8_t reg);
void PMW3901_WriteReg(uint8_t reg, uint8_t data);

uint8_t PMW3901_ReadID(void);
uint8_t PMW3901_ReadMotion(pmw3901_t *flow);
/* 异步读取 motion burst */
uint8_t PMW3901_ReadMotion_Async(pmw3901_t *flow);

/* 放在 while(1) 里一直调用 */
void PMW3901_Task(void);

/* 判断异步读取是否完成 */
uint8_t PMW3901_ReadMotion_IsDone(void);

/* 判断 PMW3901 状态机是否忙 */
uint8_t PMW3901_IsBusy(void);
void PMW3901_GetAsyncRaw(uint8_t *buf);
#endif
