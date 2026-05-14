#include "mpu9250_calib.h"
#include "bsp_timer.h"
#include <string.h>

/*
 * 设计思路：
 * - 初始化阶段仍由 MPU9250_Init() 完成。
 * - 校准阶段由 MPU9250_Calib_Task() 非阻塞地反复发起 DMA 读取。
 * - DMA 回调只置标志，真正累加和计算放在普通任务上下文里。
 * - 默认推荐 GYRO_ONLY，因为无人机只要静止即可，不要求水平。
 */

#define MPU9250_CALIB_MIN_SAMPLES        50U
#define MPU9250_CALIB_DEFAULT_PERIOD_MS  5U

static mpu9250_calib_t s_calib;

static volatile uint8_t s_waiting_dma = 0;
static volatile uint8_t s_dma_done = 0;
static volatile int     s_dma_result = 0;

static uint32_t s_last_request_ms = 0;

static int64_t s_sum_ax = 0;
static int64_t s_sum_ay = 0;
static int64_t s_sum_az = 0;
static int64_t s_sum_gx = 0;
static int64_t s_sum_gy = 0;
static int64_t s_sum_gz = 0;

static mpu9250_calib_state_t s_state = MPU9250_CALIB_IDLE;

static float MPU9250_AccelLsbPerG(mpu9250_accel_fs_t fs)
{
    switch (fs) {
        case ACCEL_FS_2:  return 16384.0f;
        case ACCEL_FS_4:  return 8192.0f;
        case ACCEL_FS_8:  return 4096.0f;
        case ACCEL_FS_16: return 2048.0f;
        default:          return 8192.0f;
    }
}

static float MPU9250_GyroLsbPerDps(mpu9250_gyro_fs_t fs)
{
    switch (fs) {
        case GYRO_FS_250:  return 131.0f;
        case GYRO_FS_500:  return 65.5f;
        case GYRO_FS_1000: return 32.8f;
        case GYRO_FS_2000: return 16.4f;
        default:           return 65.5f;
    }
}

static void MPU9250_Calib_DmaCallback(int result, const mpu9250_raw_t *data)
{
    (void)data;

    s_dma_result = result;
    s_dma_done = 1;
    s_waiting_dma = 0;
}

void MPU9250_Calib_Init(mpu9250_gyro_fs_t gyro_fs, mpu9250_accel_fs_t accel_fs)
{
    memset(&s_calib, 0, sizeof(s_calib));

    s_calib.accel_lsb_per_g = MPU9250_AccelLsbPerG(accel_fs);
    s_calib.gyro_lsb_per_dps = MPU9250_GyroLsbPerDps(gyro_fs);
    s_calib.mode = MPU9250_CALIB_MODE_GYRO_ONLY;

    s_waiting_dma = 0;
    s_dma_done = 0;
    s_dma_result = 0;
    s_last_request_ms = 0;

    s_state = MPU9250_CALIB_IDLE;
}

uint8_t MPU9250_Calib_Start(uint16_t samples, mpu9250_calib_mode_t mode)
{
    if (s_state == MPU9250_CALIB_RUNNING) {
        return 1;
    }

    if (samples < MPU9250_CALIB_MIN_SAMPLES) {
        return 2;
    }

    {
        float saved_accel_lsb = s_calib.accel_lsb_per_g;
        float saved_gyro_lsb = s_calib.gyro_lsb_per_dps;

        if (saved_accel_lsb <= 0.0f) {
            saved_accel_lsb = 8192.0f;   /* 默认 ACCEL_FS_4 */
        }
        if (saved_gyro_lsb <= 0.0f) {
            saved_gyro_lsb = 65.5f;      /* 默认 GYRO_FS_500 */
        }

        memset(&s_calib, 0, sizeof(s_calib));
        s_calib.accel_lsb_per_g = saved_accel_lsb;
        s_calib.gyro_lsb_per_dps = saved_gyro_lsb;
    }

    s_calib.samples_target = samples;
    s_calib.samples_collected = 0;
    s_calib.calibrated = 0;
    s_calib.last_error = 0;
    s_calib.mode = mode;

    s_sum_ax = 0;
    s_sum_ay = 0;
    s_sum_az = 0;
    s_sum_gx = 0;
    s_sum_gy = 0;
    s_sum_gz = 0;

    s_waiting_dma = 0;
    s_dma_done = 0;
    s_dma_result = 0;
    s_last_request_ms = 0;

    s_state = MPU9250_CALIB_RUNNING;
    return 0;
}

/*
 * 这个函数只在普通任务上下文中调用，不在中断中调用。
 */
void MPU9250_Calib_Task(void)
{
    const mpu9250_raw_t *raw;
    uint32_t now;

    if (s_state != MPU9250_CALIB_RUNNING) {
        return;
    }

    now = GetTick();

    /* 处理 DMA 完成的数据 */
    if (s_dma_done) {
        s_dma_done = 0;

        if (s_dma_result != 0) {
            s_calib.last_error = (int8_t)s_dma_result;
            s_state = MPU9250_CALIB_ERROR;
            return;
        }

        raw = MPU9250_GetRawData();
        if (raw != 0 && raw->valid) {
            s_sum_ax += raw->ax;
            s_sum_ay += raw->ay;
            s_sum_az += raw->az;
            s_sum_gx += raw->gx;
            s_sum_gy += raw->gy;
            s_sum_gz += raw->gz;

            s_calib.samples_collected++;
        }

        if (s_calib.samples_collected >= s_calib.samples_target) {
            int32_t avg_ax;
            int32_t avg_ay;
            int32_t avg_az;
            int32_t avg_gx;
            int32_t avg_gy;
            int32_t avg_gz;
            int32_t one_g;

            avg_ax = (int32_t)(s_sum_ax / (int64_t)s_calib.samples_collected);
            avg_ay = (int32_t)(s_sum_ay / (int64_t)s_calib.samples_collected);
            avg_az = (int32_t)(s_sum_az / (int64_t)s_calib.samples_collected);

            avg_gx = (int32_t)(s_sum_gx / (int64_t)s_calib.samples_collected);
            avg_gy = (int32_t)(s_sum_gy / (int64_t)s_calib.samples_collected);
            avg_gz = (int32_t)(s_sum_gz / (int64_t)s_calib.samples_collected);

            /*
             * 陀螺仪静止时理论值是 0，所以平均值就是零偏。
             */
            s_calib.gx_offset = avg_gx;
            s_calib.gy_offset = avg_gy;
            s_calib.gz_offset = avg_gz;

            /*
             * 加速度计的单姿态校准有前提：
             * - GYRO_ONLY：不改加速度零偏，避免把倾斜角误当成零偏。
             * - GYRO_ACCEL_LEVEL：假设板子水平，Z轴承受 ±1g。
             */
            if (s_calib.mode == MPU9250_CALIB_MODE_GYRO_ACCEL_LEVEL) {
                one_g = (int32_t)(s_calib.accel_lsb_per_g + 0.5f);
                if (avg_az < 0) {
                    one_g = -one_g;
                }

                s_calib.ax_offset = avg_ax;
                s_calib.ay_offset = avg_ay;
                s_calib.az_offset = avg_az - one_g;
            } else {
                s_calib.ax_offset = 0;
                s_calib.ay_offset = 0;
                s_calib.az_offset = 0;
            }

            s_calib.calibrated = 1;
            s_state = MPU9250_CALIB_DONE;
            return;
        }
    }

    /* 还没有等待 DMA，且到了采样间隔，就发起下一次 DMA 读取 */
    if (!s_waiting_dma && !MPU9250_IsBusy()) {
        if ((now - s_last_request_ms) >= MPU9250_CALIB_DEFAULT_PERIOD_MS) {
            uint8_t ret;

            s_last_request_ms = now;
            s_waiting_dma = 1;
            s_dma_done = 0;
            s_dma_result = 0;

            ret = MPU9250_ReadAccelGyroTemp_DMA(MPU9250_Calib_DmaCallback);
            if (ret != 0) {
                s_waiting_dma = 0;
                /*
                 * ret=1 通常只是 MPU/I2C 忙，下一轮重试即可；
                 * ret=2 才认为是参数/总线问题。
                 */
                if (ret == 2) {
                    s_calib.last_error = -2;
                    s_state = MPU9250_CALIB_ERROR;
                }
            }
        }
    }
}

void MPU9250_Calib_Reset(void)
{
    s_waiting_dma = 0;
    s_dma_done = 0;
    s_dma_result = 0;

    s_sum_ax = 0;
    s_sum_ay = 0;
    s_sum_az = 0;
    s_sum_gx = 0;
    s_sum_gy = 0;
    s_sum_gz = 0;

    s_calib.samples_collected = 0;
    s_calib.calibrated = 0;
    s_calib.last_error = 0;

    s_state = MPU9250_CALIB_IDLE;
}

uint8_t MPU9250_Calib_IsRunning(void)
{
    return (s_state == MPU9250_CALIB_RUNNING);
}

uint8_t MPU9250_Calib_IsDone(void)
{
    return (s_state == MPU9250_CALIB_DONE && s_calib.calibrated);
}

uint16_t MPU9250_Calib_GetProgress(void)
{
    return s_calib.samples_collected;
}

uint16_t MPU9250_Calib_GetTarget(void)
{
    return s_calib.samples_target;
}

const mpu9250_calib_t *MPU9250_Calib_GetParams(void)
{
    return &s_calib;
}

void MPU9250_Calib_SetOffsets(int32_t ax_off, int32_t ay_off, int32_t az_off,
                              int32_t gx_off, int32_t gy_off, int32_t gz_off)
{
    s_calib.ax_offset = ax_off;
    s_calib.ay_offset = ay_off;
    s_calib.az_offset = az_off;

    s_calib.gx_offset = gx_off;
    s_calib.gy_offset = gy_off;
    s_calib.gz_offset = gz_off;

    s_calib.calibrated = 1;
    s_state = MPU9250_CALIB_DONE;
}

uint8_t MPU9250_GetScaledData(mpu9250_scaled_t *out)
{
    const mpu9250_raw_t *raw;

    if (out == 0) {
        return 1;
    }

    raw = MPU9250_GetRawData();
    if (raw == 0 || !raw->valid) {
        memset(out, 0, sizeof(*out));
        return 2;
    }

    out->ax_g = ((float)((int32_t)raw->ax - s_calib.ax_offset)) / s_calib.accel_lsb_per_g;
    out->ay_g = ((float)((int32_t)raw->ay - s_calib.ay_offset)) / s_calib.accel_lsb_per_g;
    out->az_g = ((float)((int32_t)raw->az - s_calib.az_offset)) / s_calib.accel_lsb_per_g;

    out->gx_dps = ((float)((int32_t)raw->gx - s_calib.gx_offset)) / s_calib.gyro_lsb_per_dps;
    out->gy_dps = ((float)((int32_t)raw->gy - s_calib.gy_offset)) / s_calib.gyro_lsb_per_dps;
    out->gz_dps = ((float)((int32_t)raw->gz - s_calib.gz_offset)) / s_calib.gyro_lsb_per_dps;

    out->temp_c = raw->temp_c;
    out->stamp_ms = raw->stamp_ms;
    out->valid = raw->valid;

    return 0;
}
