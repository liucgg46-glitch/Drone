#include "stm32f4xx.h"
#include "bsp_uart.h"
#include "bsp_pwm.h"
#include <stdint.h>
#include <string.h>

static void SoftDelay(volatile uint32_t t)
{
    while (t--)
    {
        __NOP();
    }
}

static void UART1_SendString(const char *s)
{
    UART1_SendData_NonBlocking((uint8_t *)s, (uint16_t)strlen(s));
}

int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    UART1_Init();
    PWM_Init();

    UART1_SendString("\r\nPWM test start\r\n");

    /*
     * 第一步：固定 1000us。
     * 这时电调应该是最低油门，电机不转。
     */
    PWM_SetAllMotorUs(1100, 1000, 1000, 1000);
    UART1_SendString("PWM all = 1000us\r\n");

    while (1)
    {
        SoftDelay(12000000);
    }
}