#ifndef __BSP_RC_PWM_H
#define __BSP_RC_PWM_H

#include "stm32f4xx.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RC_PWM_CH_NUM          5

#define RC_PWM_MIN_US          900
#define RC_PWM_MAX_US          2100
#define RC_PWM_VALID_MIN_US    950
#define RC_PWM_VALID_MAX_US    2050
#define RC_PWM_TIMEOUT_MS      100

typedef struct {
    uint16_t ch[RC_PWM_CH_NUM];          /* CH1~CH5 脉宽，单位 us */
    uint8_t  valid[RC_PWM_CH_NUM];       /* 每个通道是否有效 */
    uint32_t stamp_ms[RC_PWM_CH_NUM];    /* 最近更新时间 */
} rc_pwm_data_t;

void RC_PWM_Init(void);

uint16_t RC_PWM_GetChannelUs(uint8_t ch);
uint8_t  RC_PWM_IsChannelValid(uint8_t ch);
uint8_t  RC_PWM_IsAllValid(void);
void     RC_PWM_GetAll(rc_pwm_data_t *out);

/* 放到 stm32f4xx_it.c 的 TIM3_IRQHandler / TIM4_IRQHandler 里调用 */
void RC_PWM_TIM3_IRQHandler(void);
void RC_PWM_TIM4_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif
