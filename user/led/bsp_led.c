#include "bsp_led.h"
#include "bsp_timer.h"

void led_init()
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    /* 1. Enable Clock */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    
    /* 2. Config GPIO Structure GPIO_Low_Speed*/
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Pin   = LED_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(LED_PORT, &GPIO_InitStructure);
}

void led_control(led_state_t status)
{
    if (status == LED_ON) {
        GPIO_ResetBits(LED_PORT, LED_PIN);
    } else {
        GPIO_SetBits(LED_PORT, LED_PIN);
    }
}

/* LED 任务：每 500ms 翻转一次 LED */
void task_led_blink(void) {
	
	GPIO_ToggleBits(GPIOC, GPIO_Pin_13);
//    static nbs_timer_t timer;
//    static uint8_t started = 0;

//    if (!started) {
//        nbs_timer_start(&timer, 500);
//        started = 1;
//    }

//    if (nbs_timer_expired(&timer)) {
//        /* 假设 LED 接在 PF9，请根据实际电路修改 GPIO 和引脚 */
//        GPIO_ToggleBits(GPIOC, GPIO_Pin_13);
//        nbs_timer_start(&timer, 500);  // 重新启动定时器
//    }
}
