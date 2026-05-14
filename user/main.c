#include "bsp_timer.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "stm32f4xx.h"
#include "bsp_led.h"
#include "bsp_uart.h"
#include "bsp_i2c.h"
#include "k210_protocol.h"
#include "task_scheduler.h"
#include "app_mpu_task.h"
#include "bmp280.h"
#include "app_bmp280_task.h"
#include "app_vl53l1x_task.h"


// ---------- 外部函数声明 ----------
extern void task_led_blink(void);


// ---------- 任务声明 ----------
void task_echo1(void);
void task_echo2(void);
void task_led_blink(void);
void task_debug_print(void);

// ---------- 任务结构体 ----------
task_t echo_task1 = {
    .func = task_echo1,
    .interval_ms = 5,
    .last_run = 0
};
task_t echo_task2 = {
    .func = task_echo2,
    .interval_ms = 5,
    .last_run = 0
};
task_t task_led = {
    .func = task_led_blink,
    .interval_ms = 500,
    .last_run = 0
};
task_t task_debug = {
    .func = task_debug_print,
    .interval_ms = 200,
    .last_run = 0
};

// ---------- 原有任务 ----------
void task_echo1(void) {
    uint8_t ch;
    while (UART1_GetChar(&ch)) {
        UART1_SendData_NonBlocking(&ch, 1);
    }
}

void task_echo2(void) {
    K210_ParseTask();
}

void task_debug_print(void) {
    static uint32_t last_print = 0;
    const k210_data_t *data;
    char buf[64];

    if (GetTick() - last_print < 200) return;
    last_print = GetTick();

    data = K210_GetData();
    if (data->line_valid) {
        snprintf(buf, sizeof(buf), "LINE: offset=%d\r\n", (int)data->line_offset);
        UART1_SendData_NonBlocking((uint8_t*)buf, strlen(buf));
    }
    if (data->qr_valid) {
        snprintf(buf, sizeof(buf), "QR: x=%d y=%d\r\n", (int)data->qr_x, (int)data->qr_y);
        UART1_SendData_NonBlocking((uint8_t*)buf, strlen(buf));
    }
    if (!data->line_valid && !data->qr_valid) {
        UART1_SendData_NonBlocking((uint8_t*)"No valid data\r\n", 15);
    }
}


// ---------- 主函数 ----------
int main(void)
{
    uint8_t ret;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    led_init();
    UART1_Init();
    UART2_Init();
    K210_Init();
    I2C1_Init();

    if (SysTick_Config(SystemCoreClock / 1000)) {
        while (1);
    }
    __enable_irq();
	
	scheduler_register(&task_led);
    scheduler_register(&echo_task1);
	
//====================BMP读取数据测试==============================
    UART1_SendData_NonBlocking((uint8_t*)"Ding...\r\n", 9);

    if (App_BMP280_Init() != BMP280_OK) {
        while (1) {
            /* BMP280 初始化失败，先停在这里排查接线/地址 */
        }
    }

    App_BMP280_RegisterTasks();

	
//====================MPU读取数据测试==============================	
    /* 先初始化 MPU，再启动校准 */
    ret = App_MPU9250_InitAndCalibrate(GYRO_FS_500,
                                       ACCEL_FS_4,
                                       500,
                                       MPU9250_CALIB_MODE_GYRO_ACCEL_LEVEL);
    if (ret == 0) {
        UART1_SendData_NonBlocking((uint8_t*)"MPU9250 init OK, calibrating...\r\n", 32);
    } else {
        UART1_SendData_NonBlocking((uint8_t*)"MPU9250 init/calib fail\r\n", 26);
        while (1);
    }

    /* 注册 I2C timeout、校准、DMA读取、调试打印 */
    App_MPU9250_RegisterTasks();
	
//=====================VL53L1X测试=============================		
	

	   /* 只测试一次 ID */
  if (App_VL53L1_Init() != APP_VL53L1_OK) {
    UART1_SendData_NonBlocking((uint8_t*)"VL53L1 init fail\r\n", 19);
    while (1);
}
  
	App_VL53L1_RegisterTasks();

	
//========================================================
    while (1) {
        scheduler_run();
    }
}
