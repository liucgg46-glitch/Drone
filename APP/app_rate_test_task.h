#ifndef __APP_RATE_TEST_TASK_H
#define __APP_RATE_TEST_TASK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void App_RateTest_Init(void);
void App_RateTest_RegisterTasks(void);
void App_RateTest_SetDebugPrint(uint8_t enable);

#ifdef __cplusplus
}
#endif

#endif