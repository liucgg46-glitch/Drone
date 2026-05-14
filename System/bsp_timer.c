#include "bsp_timer.h"

/* 全局毫秒计数器，在 SysTick 中断中递增 */
volatile uint32_t sys_tick = 0;

uint32_t GetTick(void)
{
    return sys_tick;
}

void nbs_timer_start(nbs_timer_t *timer, uint32_t ms)
{
    if (timer == 0) {
        return;
    }

    timer->start_time = GetTick();
    timer->duration = ms;
    timer->active = 1;
}

uint8_t nbs_timer_expired(nbs_timer_t *timer)
{
    if (timer == 0) {
        return 0;
    }

    if (!timer->active) {
        return 0;
    }

    if ((GetTick() - timer->start_time) >= timer->duration) {
        timer->active = 0;
        return 1;
    }

    return 0;
}