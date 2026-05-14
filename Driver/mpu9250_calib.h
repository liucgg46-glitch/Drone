#ifndef __MPU9250_CALIB_H
#define __MPU9250_CALIB_H

#include <stdint.h>
#include "mpu9250.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 校准模式：
 * 1) GYRO_ONLY：
 *    只校准陀螺仪零偏。推荐先用这个，模块只需要保持静止，不要求水平。
 *
 * 2) GYRO_ACCEL_LEVEL：
 *    校准陀螺仪零偏，同时假设模块水平放置，Z轴承受 ±1g。
 *    如果模块没有水平放置，不要用这个模式，否则会把姿态倾角误当成加速度零偏。
 */
typedef enum {
    MPU9250_CALIB_MODE_GYRO_ONLY = 0,
    MPU9250_CALIB_MODE_GYRO_ACCEL_LEVEL = 1
} mpu9250_calib_mode_t;

typedef enum {
    MPU9250_CALIB_IDLE = 0,
    MPU9250_CALIB_RUNNING,
    MPU9250_CALIB_DONE,
    MPU9250_CALIB_ERROR
} mpu9250_calib_state_t;

/* 原始零偏参数，单位仍然是 MPU9250 原始 LSB */
typedef struct {
    int32_t ax_offset;
    int32_t ay_offset;
    int32_t az_offset;

    int32_t gx_offset;
    int32_t gy_offset;
    int32_t gz_offset;

    float accel_lsb_per_g;
    float gyro_lsb_per_dps;

    uint16_t samples_target;
    uint16_t samples_collected;

    uint8_t calibrated;
    int8_t  last_error;
    mpu9250_calib_mode_t mode;
} mpu9250_calib_t;

/* 换算后的物理量 */
typedef struct {
    float ax_g;
    float ay_g;
    float az_g;

    float gx_dps;
    float gy_dps;
    float gz_dps;

    float temp_c;
    uint32_t stamp_ms;
    uint8_t valid;
} mpu9250_scaled_t;

/*
 * 初始化校准层。
 * 必须和 MPU9250_Init(gyro_fs, accel_fs) 传入的量程保持一致。
 */
void MPU9250_Calib_Init(mpu9250_gyro_fs_t gyro_fs, mpu9250_accel_fs_t accel_fs);

/*
 * 开始一次非阻塞校准。
 * samples 建议：
 *   300~500：启动较快
 *   800~1000：零偏更稳
 *
 * 返回值：
 *   0 = 成功开始
 *   1 = 当前已经在校准
 *   2 = 参数错误
 */
uint8_t MPU9250_Calib_Start(uint16_t samples, mpu9250_calib_mode_t mode);

/*
 * 周期调用，建议 1ms 或 5ms 调一次。
 * 这个函数内部会按 5ms 间隔发起 MPU9250 DMA 读取。
 */
void MPU9250_Calib_Task(void);

void MPU9250_Calib_Reset(void);

uint8_t MPU9250_Calib_IsRunning(void);
uint8_t MPU9250_Calib_IsDone(void);
uint16_t MPU9250_Calib_GetProgress(void);
uint16_t MPU9250_Calib_GetTarget(void);

const mpu9250_calib_t *MPU9250_Calib_GetParams(void);

/* 手动设置零偏：后面你可以把校准结果存 Flash，上电后直接恢复 */
void MPU9250_Calib_SetOffsets(int32_t ax_off, int32_t ay_off, int32_t az_off,
                              int32_t gx_off, int32_t gy_off, int32_t gz_off);

/*
 * 把当前 MPU9250_GetRawData() 转成 g 和 °/s。
 * 注意：它不主动触发 I2C 读取，只转换最近一次读到的数据。
 */
uint8_t MPU9250_GetScaledData(mpu9250_scaled_t *out);

#ifdef __cplusplus
}
#endif

#endif /* __MPU9250_CALIB_H */
