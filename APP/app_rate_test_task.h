#ifndef __APP_RATE_TEST_TASK_H
#define __APP_RATE_TEST_TASK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 角速度环测试任务
 *
 * 当前阶段用途：
 * 1. 从 app_attitude_task 获取 gx/gy/gz 角速度
 * 2. 做 roll_rate / pitch_rate / yaw_rate PID
 * 3. 混控得到 M1~M4
 * 4. 可选择是否真正输出到 PWM
 *
 * 注意：
 * - 调试阶段绝对不要装桨
 * - 默认 PWM 输出关闭
 */

void App_RateTest_Init(void);
void App_RateTest_RegisterTasks(void);
void App_RateTest_SetDebugPrint(uint8_t enable);

/*
 * enable = 0：不输出到 PWM，并且强制四路 1000us
 * enable = 1：把 rate PID 混控结果输出到 PWM
 */
void App_RateTest_EnablePwmOutput(uint8_t enable);

#ifdef __cplusplus
}
#endif

#endif /* __APP_RATE_TEST_TASK_H */