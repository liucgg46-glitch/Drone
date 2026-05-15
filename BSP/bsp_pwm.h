#ifndef __BSP_PWM_H
#define __BSP_PWM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * TIM1 PWM 输出：
 *
 * M1 -> PE9  / TIM1_CH1
 * M2 -> PE11 / TIM1_CH2
 * M3 -> PE13 / TIM1_CH3
 * M4 -> PE14 / TIM1_CH4
 *
 * 输出单位：us
 * 常用范围：1000us ~ 2000us
 */

#define PWM_MOTOR_MIN_US   1000
#define PWM_MOTOR_MAX_US   2000
#define PWM_MOTOR_STOP_US  1000

void PWM_Init(void);

void PWM_SetMotorUs(uint8_t motor_id, uint16_t us);
void PWM_SetAllMotorUs(uint16_t m1_us,
                       uint16_t m2_us,
                       uint16_t m3_us,
                       uint16_t m4_us);

void PWM_StopAll(void);

#endif