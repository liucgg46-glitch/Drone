#include "bsp_rc_pwm.h"
#include "bsp_timer.h"
#include <string.h>

/*
 * 接收机 PWM 输入捕获
 *
 * CH1 -> PC6  TIM3_CH1
 * CH2 -> PC7  TIM3_CH2
 * CH3 -> PC8  TIM3_CH3
 * CH4 -> PC9  TIM3_CH4
 * CH5 -> PD12 TIM4_CH1
 *
 * 定时器计数频率：1 MHz
 * 所以计数值 1000 = 1000 us
 */

static volatile uint16_t s_rc_width[RC_PWM_CH_NUM] = {0};
static volatile uint16_t s_rc_rise[RC_PWM_CH_NUM]  = {0};
static volatile uint32_t s_rc_stamp[RC_PWM_CH_NUM] = {0};

static uint16_t RC_PWM_CalcWidth(uint16_t now, uint16_t last)
{
    if (now >= last) {
        return (uint16_t)(now - last);
    } else {
        return (uint16_t)(0x10000U - last + now);
    }
}

static void RC_PWM_UpdateCapture(uint8_t idx,
                                 uint16_t capture,
                                 GPIO_TypeDef *gpio,
                                 uint16_t pin)
{
    uint16_t width;

    if (idx >= RC_PWM_CH_NUM) {
        return;
    }

    /*
     * 双边沿捕获：
     * 如果当前引脚是高电平，说明刚捕获到上升沿；
     * 如果当前引脚是低电平，说明刚捕获到下降沿。
     */
    if (GPIO_ReadInputDataBit(gpio, pin) == Bit_SET) {
        s_rc_rise[idx] = capture;
    } else {
        width = RC_PWM_CalcWidth(capture, s_rc_rise[idx]);

        if (width >= RC_PWM_MIN_US && width <= RC_PWM_MAX_US) {
            s_rc_width[idx] = width;
            s_rc_stamp[idx] = GetTick();
        }
    }
}

void RC_PWM_Init(void)
{
    GPIO_InitTypeDef gpio;
    TIM_TimeBaseInitTypeDef tim;
    TIM_ICInitTypeDef ic;
    NVIC_InitTypeDef nvic;

    /* GPIO 时钟 */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

    /* TIM3 / TIM4 时钟 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    /* PC6~PC9 复用为 TIM3 */
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM3);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM3);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_TIM3);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_TIM3);

    gpio.GPIO_Pin   = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
    gpio.GPIO_Mode  = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd  = GPIO_PuPd_DOWN;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &gpio);

    /* PD12 复用为 TIM4_CH1 */
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);

    gpio.GPIO_Pin   = GPIO_Pin_12;
    gpio.GPIO_Mode  = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd  = GPIO_PuPd_DOWN;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &gpio);

    /*
     * 定时器计数频率设置成 1 MHz。
     * 常见 F407 工程里 TIM3/TIM4 时钟是 84 MHz，
     * Prescaler = 84 - 1 后，1 个计数 = 1 us。
     */
    TIM_TimeBaseStructInit(&tim);
    tim.TIM_Prescaler     = 84 - 1;
    tim.TIM_CounterMode   = TIM_CounterMode_Up;
    tim.TIM_Period        = 0xFFFF;
    tim.TIM_ClockDivision = TIM_CKD_DIV1;

    TIM_TimeBaseInit(TIM3, &tim);
    TIM_TimeBaseInit(TIM4, &tim);

    /* TIM3 CH1~CH4 输入捕获 */
    TIM_ICStructInit(&ic);
    ic.TIM_ICPolarity  = TIM_ICPolarity_BothEdge;
    ic.TIM_ICSelection = TIM_ICSelection_DirectTI;
    ic.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    ic.TIM_ICFilter    = 0x03;

    ic.TIM_Channel = TIM_Channel_1;
    TIM_ICInit(TIM3, &ic);

    ic.TIM_Channel = TIM_Channel_2;
    TIM_ICInit(TIM3, &ic);

    ic.TIM_Channel = TIM_Channel_3;
    TIM_ICInit(TIM3, &ic);

    ic.TIM_Channel = TIM_Channel_4;
    TIM_ICInit(TIM3, &ic);

    /* TIM4 CH1 输入捕获 */
    ic.TIM_Channel = TIM_Channel_1;
    TIM_ICInit(TIM4, &ic);

    /* 开启捕获中断 */
    TIM_ITConfig(TIM3,
                 TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4,
                 ENABLE);

    TIM_ITConfig(TIM4,
                 TIM_IT_CC1,
                 ENABLE);

    /* NVIC TIM3 */
    nvic.NVIC_IRQChannel = TIM3_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 2;
    nvic.NVIC_IRQChannelSubPriority = 2;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    /* NVIC TIM4 */
    nvic.NVIC_IRQChannel = TIM4_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 2;
    nvic.NVIC_IRQChannelSubPriority = 3;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    memset((void *)s_rc_width, 0, sizeof(s_rc_width));
    memset((void *)s_rc_rise, 0, sizeof(s_rc_rise));
    memset((void *)s_rc_stamp, 0, sizeof(s_rc_stamp));

    TIM_Cmd(TIM3, ENABLE);
    TIM_Cmd(TIM4, ENABLE);
}

void RC_PWM_TIM3_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET) {
        TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
        RC_PWM_UpdateCapture(0, TIM_GetCapture1(TIM3), GPIOC, GPIO_Pin_6);
    }

    if (TIM_GetITStatus(TIM3, TIM_IT_CC2) != RESET) {
        TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);
        RC_PWM_UpdateCapture(1, TIM_GetCapture2(TIM3), GPIOC, GPIO_Pin_7);
    }

    if (TIM_GetITStatus(TIM3, TIM_IT_CC3) != RESET) {
        TIM_ClearITPendingBit(TIM3, TIM_IT_CC3);
        RC_PWM_UpdateCapture(2, TIM_GetCapture3(TIM3), GPIOC, GPIO_Pin_8);
    }

    if (TIM_GetITStatus(TIM3, TIM_IT_CC4) != RESET) {
        TIM_ClearITPendingBit(TIM3, TIM_IT_CC4);
        RC_PWM_UpdateCapture(3, TIM_GetCapture4(TIM3), GPIOC, GPIO_Pin_9);
    }
}

void RC_PWM_TIM4_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM4, TIM_IT_CC1) != RESET) {
        TIM_ClearITPendingBit(TIM4, TIM_IT_CC1);
        RC_PWM_UpdateCapture(4, TIM_GetCapture1(TIM4), GPIOD, GPIO_Pin_12);
    }
}

uint16_t RC_PWM_GetChannelUs(uint8_t ch)
{
    uint16_t value = 0;

    if (ch < 1 || ch > RC_PWM_CH_NUM) {
        return 0;
    }

    __disable_irq();
    value = s_rc_width[ch - 1];
    __enable_irq();

    return value;
}

uint8_t RC_PWM_IsChannelValid(uint8_t ch)
{
    uint16_t width;
    uint32_t stamp;
    uint32_t now;

    if (ch < 1 || ch > RC_PWM_CH_NUM) {
        return 0;
    }

    __disable_irq();
    width = s_rc_width[ch - 1];
    stamp = s_rc_stamp[ch - 1];
    __enable_irq();

    now = GetTick();

    if (width < RC_PWM_VALID_MIN_US || width > RC_PWM_VALID_MAX_US) {
        return 0;
    }

    if ((now - stamp) > RC_PWM_TIMEOUT_MS) {
        return 0;
    }

    return 1;
}

uint8_t RC_PWM_IsAllValid(void)
{
    uint8_t i;

    for (i = 1; i <= RC_PWM_CH_NUM; i++) {
        if (!RC_PWM_IsChannelValid(i)) {
            return 0;
        }
    }

    return 1;
}

void RC_PWM_GetAll(rc_pwm_data_t *out)
{
    uint8_t i;
    uint32_t now;

    if (out == 0) {
        return;
    }

    now = GetTick();

    __disable_irq();

    for (i = 0; i < RC_PWM_CH_NUM; i++) {
        out->ch[i] = s_rc_width[i];
        out->stamp_ms[i] = s_rc_stamp[i];

        if (s_rc_width[i] >= RC_PWM_VALID_MIN_US &&
            s_rc_width[i] <= RC_PWM_VALID_MAX_US &&
            (now - s_rc_stamp[i]) <= RC_PWM_TIMEOUT_MS) {
            out->valid[i] = 1;
        } else {
            out->valid[i] = 0;
        }
    }

    __enable_irq();
}
