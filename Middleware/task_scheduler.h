#ifndef __TASK_SCHEDULER_H
#define __TASK_SCHEDULER_H

#include <stdint.h>

/* 任务函数指针：每个任务必须是这种无参数、无返回值的函数 */
typedef void (*task_func_t)(void);

/* 任务结构体：记录每个函数和它的调度信息 */
typedef struct {
    task_func_t func;          // 任务函数指针
    uint32_t interval_ms;      // 调用间隔（毫秒）
    uint32_t last_run;         // 上次运行的时刻（毫秒计数）
} task_t;

/* 向调度器注册一个任务 */
void scheduler_register(task_t *task);

/* 调度器主循环：在主函数的 while(1) 中不断调用 */
void scheduler_run(void);

#endif
