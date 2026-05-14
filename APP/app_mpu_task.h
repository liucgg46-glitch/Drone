#ifndef __APP_MPU_TASK_H
#define __APP_MPU_TASK_H

#include <stdint.h>
#include "mpu9250.h"
#include "mpu9250_calib.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * MPU9250 应用任务层：
 * - 把 I2C 超时维护、MPU 校准、DMA 周期读取、调试打印统一封装。
 * - main.c 只需要调用 App_MPU9250_InitAndCalibrate() 和 App_MPU9250_RegisterTasks()。
 */

/*
 * 初始化 MPU9250 并启动静止校准。
 * 返回值：
 *   0      = 成功
 *   1~99   = MPU9250_Init() 返回的错误码
 *   100+x  = MPU9250_Calib_Start() 返回的错误码
 */
uint8_t App_MPU9250_InitAndCalibrate(mpu9250_gyro_fs_t gyro_fs,
                                     mpu9250_accel_fs_t accel_fs,
                                     uint16_t calib_samples,
                                     mpu9250_calib_mode_t calib_mode);

/* 注册 MPU 相关任务到调度器 */
void App_MPU9250_RegisterTasks(void);

/* 查询校准是否完成，完成后才建议接姿态解算/PID */
uint8_t App_MPU9250_IsReady(void);

/* 可选：开关串口调试打印。默认开启。 */
void App_MPU9250_SetDebugPrint(uint8_t enable);

#ifdef __cplusplus
}
#endif

#endif /* __APP_MPU_TASK_H */
