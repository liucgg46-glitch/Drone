#ifndef __APP_PMW3901_TASK_H
#define __APP_PMW3901_TASK_H

#include <stdint.h>
#include "pmw3901.h"

#ifdef __cplusplus
extern "C" {
#endif

#define APP_PMW3901_OK      0U
#define APP_PMW3901_FAIL    1U

uint8_t App_PMW3901_Init(void);
void App_PMW3901_RegisterTasks(void);

void App_PMW3901_Task(void);
void App_PMW3901_PrintTask(void);

const pmw3901_t *App_PMW3901_GetData(void);
uint8_t App_PMW3901_DataValid(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_PMW3901_TASK_H */
