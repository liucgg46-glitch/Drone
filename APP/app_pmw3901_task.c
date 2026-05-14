#include "app_pmw3901_task.h"
#include "task_scheduler.h"
#include "bsp_uart.h"
#include "bsp_timer.h"
#include <stdio.h>
#include <string.h>

static pmw3901_t g_pmw_data;
static uint8_t g_pmw_raw[12];
static uint8_t g_pmw_inited = 0;
static uint8_t g_pmw_started = 0;

static task_t g_task_pmw_read = {
    .func = App_PMW3901_Task,
    .interval_ms = 1,
    .last_run = 0
};

static task_t g_task_pmw_print = {
    .func = App_PMW3901_PrintTask,
    .interval_ms = 100,
    .last_run = 0
};

uint8_t App_PMW3901_Init(void)
{
    char buf[96];
    uint8_t id;
    uint8_t rev;

    memset(&g_pmw_data, 0, sizeof(g_pmw_data));
    memset(g_pmw_raw, 0, sizeof(g_pmw_raw));

    if (PMW3901_Init() != PMW3901_OK) {
        id = PMW3901_ReadID();
        snprintf(buf, sizeof(buf),
                 "PMW3901 init fail: product=0x%02X\r\n", id);
        UART1_SendData_NonBlocking((uint8_t*)buf, strlen(buf));
        return APP_PMW3901_FAIL;
    }

    id = PMW3901_ReadID();
    rev = PMW3901_ReadRevisionID();

    snprintf(buf, sizeof(buf),
             "PMW3901 init OK: product=0x%02X revision=0x%02X async-state\r\n",
             id, rev);
    UART1_SendData_NonBlocking((uint8_t*)buf, strlen(buf));

    g_pmw_inited = 1;
    g_pmw_started = 0;
    return APP_PMW3901_OK;
}

void App_PMW3901_RegisterTasks(void)
{
    scheduler_register(&g_task_pmw_read);
    scheduler_register(&g_task_pmw_print);
}

void App_PMW3901_Task(void)
{
    static uint32_t last_start = 0;
    uint32_t now;

    if (!g_pmw_inited) {
        return;
    }

    /* 推进 PMW3901 内部异步状态机 */
    PMW3901_Task();

    if (PMW3901_ReadMotion_IsDone()) {
        PMW3901_GetAsyncRaw(g_pmw_raw);
        g_pmw_started = 0;
    }

    now = GetTick();

    /*
     * 50Hz 光流读取，调试阶段够用。
     * 稳定后可以改 10ms 或更快。
     */
    if (!g_pmw_started && !PMW3901_IsBusy()) {
        if ((now - last_start) >= 20U) {
            last_start = now;
            if (PMW3901_ReadMotion_Async(&g_pmw_data) == PMW3901_OK) {
                g_pmw_started = 1;
            }
        }
    }
}

void App_PMW3901_PrintTask(void)
{
    char buf[192];

    if (!g_pmw_inited) {
        return;
    }

    snprintf(buf, sizeof(buf),
             "PMW dx:%6d dy:%6d mot:0x%02X m:%u ovf:%u squal:%3u shutter:%5u valid:%u raw:%02X %02X %02X %02X %02X %02X age:%lu ms\r\n",
             g_pmw_data.dx,
             g_pmw_data.dy,
             g_pmw_data.motion,
             g_pmw_data.motion_detected,
             g_pmw_data.overflow,
             g_pmw_data.quality,
             g_pmw_data.shutter,
             g_pmw_data.valid,
             g_pmw_raw[0], g_pmw_raw[1], g_pmw_raw[2],
             g_pmw_raw[3], g_pmw_raw[4], g_pmw_raw[5],
             (unsigned long)(GetTick() - g_pmw_data.stamp_ms));

    UART1_SendData_NonBlocking((uint8_t*)buf, strlen(buf));
}

const pmw3901_t *App_PMW3901_GetData(void)
{
    return &g_pmw_data;
}

uint8_t App_PMW3901_DataValid(void)
{
    return g_pmw_data.valid;
}
