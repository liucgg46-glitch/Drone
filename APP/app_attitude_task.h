#ifndef __APP_ATTITUDE_TASK_H
#define __APP_ATTITUDE_TASK_H

#include <stdint.h>
#include "mahony.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 坐标轴方向映射。
 * 目标：送进 Mahony 的静止水平加速度最好接近 ax=0, ay=0, az=+1。
 *
 * 第一次测试先不要乱改。若你水平静止时 MPU 输出 az≈-1，
 * 可先把 APP_ATTITUDE_ACC_Z_SIGN 改成 -1.0f，再观察 roll/pitch 是否正常。
 * gyro 的符号要根据实际倾斜/旋转方向再微调。
 */
#define APP_ATTITUDE_ACC_X_SIGN      1.0f
#define APP_ATTITUDE_ACC_Y_SIGN      1.0f
#define APP_ATTITUDE_ACC_Z_SIGN      1.0f

#define APP_ATTITUDE_GYRO_X_SIGN     1.0f
#define APP_ATTITUDE_GYRO_Y_SIGN     1.0f
#define APP_ATTITUDE_GYRO_Z_SIGN     1.0f

typedef struct {
    float roll_deg;
    float pitch_deg;
    float yaw_deg;

    float ax_g;
    float ay_g;
    float az_g;

    float gx_dps;
    float gy_dps;
    float gz_dps;

    float dt_s;
    uint32_t stamp_ms;
    uint8_t valid;
} app_attitude_data_t;

void App_Attitude_Init(void);
void App_Attitude_RegisterTasks(void);
void App_Attitude_SetDebugPrint(uint8_t enable);
const app_attitude_data_t *App_Attitude_GetData(void);
const mahony_t *App_Attitude_GetMahony(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_ATTITUDE_TASK_H */
