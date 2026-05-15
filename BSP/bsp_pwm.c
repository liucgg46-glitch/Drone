#include "bsp_pwm.h"
#include "stm32f4xx.h"

static uint16_t PWM_ClampUs(uint16_t us)
{
    if (us < PWM_MOTOR_MIN_US)
    {
        return PWM_MOTOR_MIN_US;
    }

    if (us > PWM_MOTOR_MAX_US)
    {
        return PWM_MOTOR_MAX_US;
    }

    return us;
}

void PWM_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    /*
     * GPIOE 时钟
     * TIM1 时钟
     */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

    /*
     * PE9  -> TIM1_CH1
     * PE11 -> TIM1_CH2
     * PE13 -> TIM1_CH3
     * PE14 -> TIM1_CH4
     */
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource9,  GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_TIM1);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_11 | GPIO_Pin_13 | GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    /*
     * 假设 TIM1 时钟 = 168MHz
     * 168MHz / 168 = 1MHz
     * 1 个计数 = 1us
     *
     * ARR = 20000 - 1
     * 周期 = 20ms = 50Hz
     *
     * CCR = 1000 -> 1000us
     * CCR = 1500 -> 1500us
     * CCR = 2000 -> 2000us
     */
    TIM_TimeBaseStructure.TIM_Prescaler = 168 - 1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period = 20000 - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

    TIM_OCStructInit(&TIM_OCInitStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_OCInitStructure.TIM_Pulse = PWM_MOTOR_STOP_US;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;

    TIM_OC1Init(TIM1, &TIM_OCInitStructure);
    TIM_OC2Init(TIM1, &TIM_OCInitStructure);
    TIM_OC3Init(TIM1, &TIM_OCInitStructure);
    TIM_OC4Init(TIM1, &TIM_OCInitStructure);

    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM1, ENABLE);

    /*
     * TIM1 是高级定时器，必须打开主输出
     */
    TIM_CtrlPWMOutputs(TIM1, ENABLE);

    TIM_Cmd(TIM1, ENABLE);

    PWM_StopAll();
}

void PWM_SetMotorUs(uint8_t motor_id, uint16_t us)
{
    us = PWM_ClampUs(us);

    switch (motor_id)
    {
        case 1:
            TIM_SetCompare1(TIM1, us);
            break;

        case 2:
            TIM_SetCompare2(TIM1, us);
            break;

        case 3:
            TIM_SetCompare3(TIM1, us);
            break;

        case 4:
            TIM_SetCompare4(TIM1, us);
            break;

        default:
            break;
    }
}

void PWM_SetAllMotorUs(uint16_t m1_us,
                       uint16_t m2_us,
                       uint16_t m3_us,
                       uint16_t m4_us)
{
    PWM_SetMotorUs(1, m1_us);
    PWM_SetMotorUs(2, m2_us);
    PWM_SetMotorUs(3, m3_us);
    PWM_SetMotorUs(4, m4_us);
}

void PWM_StopAll(void)
{
    PWM_SetAllMotorUs(PWM_MOTOR_STOP_US,
                      PWM_MOTOR_STOP_US,
                      PWM_MOTOR_STOP_US,
                      PWM_MOTOR_STOP_US);
}