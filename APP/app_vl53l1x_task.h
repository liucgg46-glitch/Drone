#ifndef __APP_VL53L1X_TASK_H
#define __APP_VL53L1X_TASK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    APP_VL53L1_OK = 0,
    APP_VL53L1_ERR = 1
} app_vl53l1_status_t;

typedef struct {
    uint16_t distance_mm;
    uint8_t  range_status;
    uint32_t stamp_ms;
    uint8_t  valid;
} app_vl53l1_data_t;

app_vl53l1_status_t App_VL53L1_Init(void);
void App_VL53L1_RegisterTasks(void);
void App_VL53L1_RangingTask(void);
void App_VL53L1_PrintTask(void);

const app_vl53l1_data_t *App_VL53L1_GetData(void);

#ifdef __cplusplus
}
#endif

#endif
