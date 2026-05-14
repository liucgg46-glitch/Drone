#ifndef __BSP_TIMER_H
#define __BSP_TIMER_H

#include <stdint.h>

/* 非阻塞定时器结构体 */
typedef struct {
    uint32_t start_time;
    uint32_t duration;
    uint8_t  active;
} nbs_timer_t;

/* 获取系统毫秒计数 */
uint32_t GetTick(void);

/* 启动非阻塞定时器 */
void nbs_timer_start(nbs_timer_t *timer, uint32_t ms);

/* 检查定时器是否超时（超时返回1，否则0） */
uint8_t nbs_timer_expired(nbs_timer_t *timer);

#endif
