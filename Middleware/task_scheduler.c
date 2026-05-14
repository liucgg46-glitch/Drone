#include "task_scheduler.h"
#include "bsp_timer.h"          // 使用 GetTick()
#include <stdio.h>

#define MAX_TASKS 16            // 最多支持16个任务

static task_t *task_list[MAX_TASKS];  // 任务指针数组
static uint8_t task_count = 0;         // 当前任务数量

/**
 * @brief  注册一个任务到调度器列表
 * @param  task: 指向任务结构体的指针（通常定义为全局或静态变量）
 * @retval 无
 */
void scheduler_register(task_t *task)
{
    uint8_t i;

    if (task == NULL) return;

    /* 防重复注册 */
    for (i = 0; i < task_count; i++) {
        if (task_list[i] == task) {
            return;
        }
    }

    if (task_count < MAX_TASKS) {
        task_list[task_count] = task;
        task_count++;
    }
}

/**
 * @brief  调度器主循环：遍历所有任务，对到期的任务进行调用
 * @param  无
 * @retval 无
 * @note   调用时机：必须放在主函数的 while(1) 中，以最小开销循环调用
 */
void scheduler_run(void)
{
    uint32_t now = GetTick();
    uint8_t i;

    for (i = 0; i < task_count; i++) {
        task_t *t = task_list[i];

        if (t == NULL) continue;

        if ((now - t->last_run) >= t->interval_ms) {

            /* 防止周期漂移 */
            t->last_run += t->interval_ms;

            if (t->func != NULL) {
                t->func();
            }
        }
    }
}
