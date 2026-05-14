#include "app_vl53l1x_task.h"

#include "vl53l1_api.h"
#include "vl53l1_platform.h"
#include "task_scheduler.h"
#include "bsp_uart.h"
#include "bsp_timer.h"
#include "bsp_i2c.h"

#include <stdio.h>
#include <string.h>

#define APP_VL53L1_ADDR_8BIT      0x52U
#define APP_VL53L1_BUDGET_US      50000UL
#define APP_VL53L1_PERIOD_MS      100UL

/* 常用寄存器：用于正式 API 初始化前先确认 platform 读写是否正常 */
#define VL53L1_REG_IDENTIFICATION_MODEL_ID   0x010FU
#define VL53L1_REG_FIRMWARE_SYSTEM_STATUS    0x00E5U

static VL53L1_Dev_t g_vl53_dev;
static app_vl53l1_data_t g_vl53_data;
static uint8_t g_vl53_started = 0;

static void App_VL53L1_PrintError(const char *step, VL53L1_Error err)
{
    char buf[96];
    snprintf(buf, sizeof(buf), "VL53L1 %s fail, err=%d\r\n", step, (int)err);
    UART1_SendData_NonBlocking((uint8_t*)buf, strlen(buf));
}

static void App_VL53L1_Precheck(void)
{
    char buf[128];
    uint16_t id = 0;
    uint8_t boot = 0;
    VL53L1_Error err_id;
    VL53L1_Error err_boot;

    err_id = VL53L1_RdWord(&g_vl53_dev, VL53L1_REG_IDENTIFICATION_MODEL_ID, &id);
    err_boot = VL53L1_RdByte(&g_vl53_dev, VL53L1_REG_FIRMWARE_SYSTEM_STATUS, &boot);

    snprintf(buf, sizeof(buf),
             "VL53L1 precheck: id_err=%d id=0x%04X boot_err=%d boot=0x%02X\r\n",
             (int)err_id, id, (int)err_boot, boot);
    UART1_SendData_NonBlocking((uint8_t*)buf, strlen(buf));
}

app_vl53l1_status_t App_VL53L1_Init(void)
{
    VL53L1_Error err;
    char buf[128];

    memset(&g_vl53_dev, 0, sizeof(g_vl53_dev));
    memset(&g_vl53_data, 0, sizeof(g_vl53_data));

    g_vl53_dev.i2c_slave_address = APP_VL53L1_ADDR_8BIT;

    err = VL53L1_CommsInitialise(&g_vl53_dev, 1, 400);
    if (err != VL53L1_ERROR_NONE) {
        App_VL53L1_PrintError("CommsInitialise", err);
        return APP_VL53L1_ERR;
    }

    /* 让刚上电/刚拉高 XSHUT 的 VL53L1X 有一点启动时间 */
    VL53L1_WaitMs(&g_vl53_dev, 10);

    /* 先测试 platform 层能否通过官方 API 读 16 位寄存器和单字节寄存器 */
    App_VL53L1_Precheck();

    err = VL53L1_WaitDeviceBooted(&g_vl53_dev);
    if (err != VL53L1_ERROR_NONE) {
        App_VL53L1_PrintError("WaitDeviceBooted", err);
        return APP_VL53L1_ERR;
    }

    err = VL53L1_DataInit(&g_vl53_dev);
    if (err != VL53L1_ERROR_NONE) {
        App_VL53L1_PrintError("DataInit", err);
        return APP_VL53L1_ERR;
    }

    err = VL53L1_StaticInit(&g_vl53_dev);
    if (err != VL53L1_ERROR_NONE) {
        App_VL53L1_PrintError("StaticInit", err);
        return APP_VL53L1_ERR;
    }

    err = VL53L1_SetDistanceMode(&g_vl53_dev, VL53L1_DISTANCEMODE_LONG);
    if (err != VL53L1_ERROR_NONE) {
        App_VL53L1_PrintError("SetDistanceMode", err);
        return APP_VL53L1_ERR;
    }

    err = VL53L1_SetMeasurementTimingBudgetMicroSeconds(&g_vl53_dev, APP_VL53L1_BUDGET_US);
    if (err != VL53L1_ERROR_NONE) {
        App_VL53L1_PrintError("SetBudget", err);
        return APP_VL53L1_ERR;
    }

    err = VL53L1_SetInterMeasurementPeriodMilliSeconds(&g_vl53_dev, APP_VL53L1_PERIOD_MS);
    if (err != VL53L1_ERROR_NONE) {
        App_VL53L1_PrintError("SetPeriod", err);
        return APP_VL53L1_ERR;
    }

    err = VL53L1_StartMeasurement(&g_vl53_dev);
    if (err != VL53L1_ERROR_NONE) {
        App_VL53L1_PrintError("StartMeasurement", err);
        return APP_VL53L1_ERR;
    }

    g_vl53_started = 1;

    snprintf(buf, sizeof(buf),
             "VL53L1 full API ranging OK: addr=0x%02X budget=%luus period=%lums\r\n",
             APP_VL53L1_ADDR_8BIT,
             (unsigned long)APP_VL53L1_BUDGET_US,
             (unsigned long)APP_VL53L1_PERIOD_MS);
    UART1_SendData_NonBlocking((uint8_t*)buf, strlen(buf));

    return APP_VL53L1_OK;
}

void App_VL53L1_RangingTask(void)
{
    VL53L1_Error err;
    uint8_t ready = 0;
    VL53L1_RangingMeasurementData_t ranging_data;

    if (!g_vl53_started) {
        return;
    }

    if (I2C1_IsBusy()) {
        return;
    }

    err = VL53L1_GetMeasurementDataReady(&g_vl53_dev, &ready);
    if (err != VL53L1_ERROR_NONE || ready == 0) {
        return;
    }

    err = VL53L1_GetRangingMeasurementData(&g_vl53_dev, &ranging_data);
    if (err == VL53L1_ERROR_NONE) {
        g_vl53_data.distance_mm = ranging_data.RangeMilliMeter;
        g_vl53_data.range_status = ranging_data.RangeStatus;
        g_vl53_data.stamp_ms = GetTick();
        g_vl53_data.valid = 1;
    }

    (void)VL53L1_ClearInterruptAndStartMeasurement(&g_vl53_dev);
}

void App_VL53L1_PrintTask(void)
{
    char buf[96];

    if (!g_vl53_data.valid) {
        return;
    }

    snprintf(buf, sizeof(buf),
             "VL53L1 D:%5u mm status:%u age:%lu ms\r\n",
             g_vl53_data.distance_mm,
             g_vl53_data.range_status,
             (unsigned long)(GetTick() - g_vl53_data.stamp_ms));
    UART1_SendData_NonBlocking((uint8_t*)buf, strlen(buf));
}

const app_vl53l1_data_t *App_VL53L1_GetData(void)
{
    return &g_vl53_data;
}

static task_t task_vl53l1_ranging = {
    .func = App_VL53L1_RangingTask,
    .interval_ms = 10,
    .last_run = 0
};

static task_t task_vl53l1_print = {
    .func = App_VL53L1_PrintTask,
    .interval_ms = 200,
    .last_run = 0
};

void App_VL53L1_RegisterTasks(void)
{
    scheduler_register(&task_vl53l1_ranging);
    scheduler_register(&task_vl53l1_print);
}
