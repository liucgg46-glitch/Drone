#ifndef __APP_ANGLE_TEST_TASK_H
#define __APP_ANGLE_TEST_TASK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 角度外环测试任务
 *
 * 当前阶段用途：
 * 1. 目标 roll/pitch = 0 度
 * 2. 角度外环输出目标角速度
 * 3. 角速度内环输出修正量
 * 4. 混控得到 M1~M4
 * 5. 可选择是否输出到 PWM
 *
 * 注意：
 * 调试阶段绝对不要装桨。
 */

void App_AngleTest_Init(void);
void App_AngleTest_RegisterTasks(void);
void App_AngleTest_SetDebugPrint(uint8_t enable);
void App_AngleTest_EnablePwmOutput(uint8_t enable);

#endif