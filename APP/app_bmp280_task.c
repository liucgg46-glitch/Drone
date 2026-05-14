#include "app_bmp280_task.h"
#include "bmp280.h"
#include "task_scheduler.h"
#include "bsp_uart.h"
#include "bsp_i2c.h"
#include <stdio.h>
#include <string.h>

static bmp280_data_t g_bmp_latest;
static volatile uint8_t g_bmp_ready = 0;

static task_t task_bmp_read_obj = {
    .func = task_bmp280_read,
    .interval_ms = 100,
    .last_run = 0
};

static task_t task_bmp_print_obj = {
    .func = task_bmp280_print,
    .interval_ms = 500,
    .last_run = 0
};

uint8_t App_BMP280_Init(void)
{
    uint8_t ret;
    char buf[96];

    ret = BMP280_Init();
    if (ret == BMP280_OK) {
        snprintf(buf, sizeof(buf),
                 "BMP280 init OK: addr=0x%02X id=0x%02X\r\n",
                 BMP280_GetAddress(), BMP280_GetChipId());
        UART1_SendData_NonBlocking((uint8_t*)buf, strlen(buf));
    } else {
        snprintf(buf, sizeof(buf), "BMP280 init fail: ret=%u\r\n", ret);
        UART1_SendData_NonBlocking((uint8_t*)buf, strlen(buf));
    }

    return ret;
}

void App_BMP280_RegisterTasks(void)
{
    scheduler_register(&task_bmp_read_obj);
    scheduler_register(&task_bmp_print_obj);
}

void task_bmp280_read(void)
{
    /* 和 MPU6500 的 I2C DMA 状态机共用 I2C1，总线忙时本次跳过，下次再读 */
    if (I2C1_IsBusy()) {
        return;
    }

    if (BMP280_ReadData(&g_bmp_latest) == BMP280_OK) {
        g_bmp_ready = 1;
    }
}

void task_bmp280_print(void)
{
    char buf[128];

    if (!g_bmp_ready) {
        UART1_SendData_NonBlocking((uint8_t*)"BMP280 no data\r\n", 16);
        return;
    }

    snprintf(buf, sizeof(buf),
             "BMP T:%6.2fC P:%8.2fPa %7.2fhPa H:%7.2fm\r\n",
             g_bmp_latest.temperature_c,
             g_bmp_latest.pressure_pa,
             g_bmp_latest.pressure_hpa,
             g_bmp_latest.altitude_m);

    UART1_SendData_NonBlocking((uint8_t*)buf, strlen(buf));
}
