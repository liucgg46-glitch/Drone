#include "app_attitude_task.h"
#include "task_scheduler.h"
#include "mpu9250.h"
#include "mpu9250_calib.h"
#include "bsp_uart.h"
#include "bsp_timer.h"
#include <stdio.h>
#include <string.h>

#define APP_ATTITUDE_DEG_TO_RAD       0.017453292519943295f
#define APP_ATTITUDE_DEFAULT_DT_S     0.005f

static mahony_t s_mahony;
static app_attitude_data_t s_att;
static uint8_t s_debug_print_enable = 1;
static uint8_t s_inited = 0;
static uint32_t s_last_imu_stamp_ms = 0;

static void App_Attitude_UpdateTask(void);
static void App_Attitude_PrintTask(void);

static task_t s_task_att_update = {
    .func = App_Attitude_UpdateTask,
    .interval_ms = 1,
    .last_run = 0
};

static task_t s_task_att_print = {
    .func = App_Attitude_PrintTask,
    .interval_ms = 100,
    .last_run = 0
};

void App_Attitude_Init(void)
{
    memset(&s_att, 0, sizeof(s_att));

    /* 先用保守参数：kp=2.0，ki=0。确认姿态方向后再开 ki。 */
    Mahony_Init(&s_mahony, 2.0f, 0.0f);

    s_last_imu_stamp_ms = 0;
    s_inited = 1;
}

void App_Attitude_RegisterTasks(void)
{
    scheduler_register(&s_task_att_update);
    scheduler_register(&s_task_att_print);
}

void App_Attitude_SetDebugPrint(uint8_t enable)
{
    s_debug_print_enable = enable ? 1U : 0U;
}

const app_attitude_data_t *App_Attitude_GetData(void)
{
    return &s_att;
}

const mahony_t *App_Attitude_GetMahony(void)
{
    return &s_mahony;
}

static float App_Attitude_CalcDt(uint32_t now_stamp_ms)
{
    float dt;

    if (s_last_imu_stamp_ms == 0 || now_stamp_ms == 0) {
        s_last_imu_stamp_ms = now_stamp_ms;
        return APP_ATTITUDE_DEFAULT_DT_S;
    }

    dt = ((float)(now_stamp_ms - s_last_imu_stamp_ms)) * 0.001f;
    s_last_imu_stamp_ms = now_stamp_ms;

    /* 防止调试暂停/异常间隔导致四元数跳变 */
    if (dt < 0.001f || dt > 0.050f) {
        dt = APP_ATTITUDE_DEFAULT_DT_S;
    }

    return dt;
}

static void App_Attitude_UpdateTask(void)
{
    mpu9250_scaled_t imu;
    float ax, ay, az;
    float gx_dps, gy_dps, gz_dps;
    float gx_rad, gy_rad, gz_rad;
    float dt;

    if (!s_inited) {
        return;
    }

    if (!MPU9250_Calib_IsDone()) {
        return;
    }

    if (!MPU9250_DataAvailable()) {
        return;
    }

    if (MPU9250_GetScaledData(&imu) != 0 || !imu.valid) {
        MPU9250_ClearDataAvailable();
        return;
    }

    MPU9250_ClearDataAvailable();

    dt = App_Attitude_CalcDt(imu.stamp_ms);

    ax = APP_ATTITUDE_ACC_X_SIGN * imu.ax_g;
    ay = APP_ATTITUDE_ACC_Y_SIGN * imu.ay_g;
    az = APP_ATTITUDE_ACC_Z_SIGN * imu.az_g;

    gx_dps = APP_ATTITUDE_GYRO_X_SIGN * imu.gx_dps;
    gy_dps = APP_ATTITUDE_GYRO_Y_SIGN * imu.gy_dps;
    gz_dps = APP_ATTITUDE_GYRO_Z_SIGN * imu.gz_dps;

    gx_rad = gx_dps * APP_ATTITUDE_DEG_TO_RAD;
    gy_rad = gy_dps * APP_ATTITUDE_DEG_TO_RAD;
    gz_rad = gz_dps * APP_ATTITUDE_DEG_TO_RAD;

    Mahony_UpdateIMU(&s_mahony,
                     gx_rad, gy_rad, gz_rad,
                     ax, ay, az,
                     dt);

    s_att.roll_deg = s_mahony.euler.roll;
    s_att.pitch_deg = s_mahony.euler.pitch;
    s_att.yaw_deg = s_mahony.euler.yaw;

    s_att.ax_g = ax;
    s_att.ay_g = ay;
    s_att.az_g = az;

    s_att.gx_dps = gx_dps;
    s_att.gy_dps = gy_dps;
    s_att.gz_dps = gz_dps;

    s_att.dt_s = dt;
    s_att.stamp_ms = imu.stamp_ms;
    s_att.valid = 1;
}

static void App_Attitude_PrintTask(void)
{
    char buf[192];

    if (!s_debug_print_enable) {
        return;
    }

    if (!s_inited) {
        return;
    }

    if (!MPU9250_Calib_IsDone()) {
        UART1_SendData_NonBlocking((uint8_t*)"ATT waiting MPU calib\r\n", 23);
        return;
    }

    if (!s_att.valid) {
        UART1_SendData_NonBlocking((uint8_t*)"ATT waiting IMU data\r\n", 22);
        return;
    }

    snprintf(buf, sizeof(buf),
             "ATT R:%8.2f P:%8.2f Y:%8.2f | A:% .2f % .2f % .2f | G:% .2f % .2f % .2f | dt:%.3f\r\n",
             s_att.roll_deg,
             s_att.pitch_deg,
             s_att.yaw_deg,
             s_att.ax_g,
             s_att.ay_g,
             s_att.az_g,
             s_att.gx_dps,
             s_att.gy_dps,
             s_att.gz_dps,
             s_att.dt_s);

    UART1_SendData_NonBlocking((uint8_t*)buf, (uint16_t)strlen(buf));
}
