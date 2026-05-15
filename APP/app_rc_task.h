#ifndef __APP_RC_TASK_H
#define __APP_RC_TASK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void App_RC_Init(void);
void App_RC_RegisterTasks(void);
void App_RC_SetDebugPrint(uint8_t enable);

/* 新增：遥控器电机测试 */
void App_RC_MotorTest_Init(void);
void App_RC_MotorTest_Enable(uint8_t enable);
uint8_t App_RC_MotorTest_IsArmed(void);

#ifdef __cplusplus
}
#endif

#endif
