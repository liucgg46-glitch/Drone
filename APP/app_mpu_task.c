#include "app_mpu_task.h"
#include "task_scheduler.h"
#include "bsp_i2c.h"
#include "bsp_uart.h"
#include <stdio.h>
#include <string.h>

/* ==================== 内部状态 ==================== */
static uint8_t s_debug_print_enable = 1;

/* ==================== 内部任务函数声明 ==================== */
static void App_Task_I2C_Timeout(void);
static void App_Task_MPU9250_Calib(void);
static void App_Task_MPU9250_DMA_Read(void);
static void App_Task_MPU9250_DebugPrint(void);

/* ==================== 任务对象 ====================
 * 注意：task_t 必须是全局/static，不能放在函数局部变量里，
 * 因为 scheduler_register() 保存的是任务结构体指针。
 */
static task_t app_task_i2c = {
    .func = App_Task_I2C_Timeout,
    .interval_ms = 1,
    .last_run = 0
};

static task_t app_task_mpu_calib = {
    .func = App_Task_MPU9250_Calib,
    .interval_ms = 1,
    .last_run = 0
};

static task_t app_task_mpu_read = {
    .func = App_Task_MPU9250_DMA_Read,
    .interval_ms = 5,
    .last_run = 0
};

static task_t app_task_mpu_print = {
    .func = App_Task_MPU9250_DebugPrint,
    .interval_ms = 200,
    .last_run = 0
};

/* ==================== 对外接口 ==================== */
uint8_t App_MPU9250_InitAndCalibrate(mpu9250_gyro_fs_t gyro_fs,
                                     mpu9250_accel_fs_t accel_fs,
                                     uint16_t calib_samples,
                                     mpu9250_calib_mode_t calib_mode)
{
    uint8_t ret;

    if (calib_samples == 0) {
        calib_samples = 500;
    }

    /* 先初始化真实硬件，再初始化校准层，再启动校准 */
    ret = MPU9250_Init(gyro_fs, accel_fs);
    if (ret != 0) {
        return ret;
    }

    MPU9250_Calib_Init(gyro_fs, accel_fs);

    ret = MPU9250_Calib_Start(calib_samples, calib_mode);
    if (ret != 0) {
        return (uint8_t)(100U + ret);
    }

    return 0;
}

void App_MPU9250_RegisterTasks(void)
{
    scheduler_register(&app_task_i2c);
    scheduler_register(&app_task_mpu_calib);
    scheduler_register(&app_task_mpu_read);
    scheduler_register(&app_task_mpu_print);
}

uint8_t App_MPU9250_IsReady(void)
{
    return MPU9250_Calib_IsDone();
}

void App_MPU9250_SetDebugPrint(uint8_t enable)
{
    s_debug_print_enable = enable ? 1U : 0U;
}

/* ==================== 内部任务实现 ==================== */
static void App_Task_I2C_Timeout(void)
{
    I2C1_Task();
}

static void App_Task_MPU9250_Calib(void)
{
    MPU9250_Calib_Task();
}

static void App_Task_MPU9250_DMA_Read(void)
{
    /* 校准未完成前，不额外发起普通读取，避免和校准任务抢 MPU/I2C */
    if (!MPU9250_Calib_IsDone()) {
        return;
    }

    if (!MPU9250_IsBusy()) {
        (void)MPU9250_ReadAccelGyroTemp_DMA(0);
    }
}

static void App_Task_MPU9250_DebugPrint(void)
{
    char buf[160];

    if (!s_debug_print_enable) {
        return;
    }

    if (MPU9250_Calib_IsRunning()) {
        snprintf(buf, sizeof(buf),
                 "MPU calib: %u/%u\r\n",
                 (unsigned)MPU9250_Calib_GetProgress(),
                 (unsigned)MPU9250_Calib_GetTarget());
        UART1_SendData_NonBlocking((uint8_t*)buf, (uint16_t)strlen(buf));
        return;
    }

    if (!MPU9250_Calib_IsDone()) {
        UART1_SendData_NonBlocking((uint8_t*)"MPU calib not done\r\n", 20);
        return;
    }

    if (!MPU9250_DataAvailable()) {
        return;
    }

    MPU9250_ClearDataAvailable();

    /* 打印校准后物理量，不再打印原始 LSB */
    {
        mpu9250_scaled_t s;
        if (MPU9250_GetScaledData(&s) != 0) {
            return;
        }

        snprintf(buf, sizeof(buf),
                 "CAL A:% .3f % .3f % .3f g  G:% .2f % .2f % .2f dps  T:%.2fC\r\n",
                 s.ax_g, s.ay_g, s.az_g,
                 s.gx_dps, s.gy_dps, s.gz_dps,
                 s.temp_c);
        UART1_SendData_NonBlocking((uint8_t*)buf, (uint16_t)strlen(buf));
    }
}
