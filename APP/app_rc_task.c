#include "app_rc_task.h"

#include "bsp_rc_pwm.h"
#include "bsp_pwm.h"
#include "bsp_uart.h"
#include "task_scheduler.h"

#include <stdio.h>
#include <string.h>

/*
 * RC 通道定义
 * CH1: Roll
 * CH2: Pitch
 * CH3: Throttle
 * CH4: Yaw
 * CH5: Arm 解锁开关
 */
#define RC_ROLL_CH              1
#define RC_PITCH_CH             2
#define RC_THROTTLE_CH          3
#define RC_YAW_CH               4
#define RC_ARM_CH               5

/*
 * 解锁逻辑
 * CH3 必须在最低位附近，才允许解锁
 * CH5 高位才允许解锁
 */
#define RC_THROTTLE_LOW_US      1050
#define RC_ARM_ON_US            1700
#define RC_ARM_OFF_US           1300

/*
 * 第一次电机测试，最大值不要太高。
 * 先限制到 1250us。
 * 确认安全后可以改成 1300、1400。
 */
#define MOTOR_MIN_US            1000
#define MOTOR_IDLE_US           1050
#define MOTOR_TEST_MAX_US       1250

static uint8_t s_rc_debug_print = 1;

/* 电机测试状态 */
static uint8_t  s_motor_test_enable = 0;
static uint8_t  s_pwm_inited = 0;
static uint8_t  s_armed = 0;
static uint16_t s_motor_out = MOTOR_MIN_US;

static void App_RC_DebugPrintTask(void);
static void App_RC_MotorTestTask(void);

static task_t app_rc_print_task = {
    .func = App_RC_DebugPrintTask,
    .interval_ms = 200,
    .last_run = 0
};

static task_t app_rc_motor_test_task = {
    .func = App_RC_MotorTestTask,
    .interval_ms = 20,
    .last_run = 0
};

static uint16_t clamp_u16_local(int32_t x, uint16_t min_v, uint16_t max_v)
{
    if (x < (int32_t)min_v) {
        return min_v;
    }

    if (x > (int32_t)max_v) {
        return max_v;
    }

    return (uint16_t)x;
}

static uint16_t map_throttle_to_motor(uint16_t ch3)
{
    int32_t out;

    if (ch3 <= RC_THROTTLE_LOW_US) {
        return MOTOR_MIN_US;
    }

    if (ch3 > 2000) {
        ch3 = 2000;
    }

    /*
     * CH3: 1050~2000
     * 映射到电机输出：1050~1250
     */
    out = MOTOR_IDLE_US +
          ((int32_t)(ch3 - RC_THROTTLE_LOW_US) *
           (MOTOR_TEST_MAX_US - MOTOR_IDLE_US)) /
          (2000 - RC_THROTTLE_LOW_US);

    return clamp_u16_local(out, MOTOR_IDLE_US, MOTOR_TEST_MAX_US);
}

void App_RC_Init(void)
{
    RC_PWM_Init();
}

void App_RC_RegisterTasks(void)
{
    scheduler_register(&app_rc_print_task);
    scheduler_register(&app_rc_motor_test_task);
}

void App_RC_SetDebugPrint(uint8_t enable)
{
    s_rc_debug_print = enable ? 1U : 0U;
}

void App_RC_MotorTest_Init(void)
{
    PWM_Init();
    PWM_StopAll();

    s_pwm_inited = 1;
    s_armed = 0;
    s_motor_out = MOTOR_MIN_US;
}

void App_RC_MotorTest_Enable(uint8_t enable)
{
    s_motor_test_enable = enable ? 1U : 0U;

    if (!s_motor_test_enable && s_pwm_inited) {
        s_armed = 0;
        s_motor_out = MOTOR_MIN_US;
        PWM_StopAll();
    }
}

uint8_t App_RC_MotorTest_IsArmed(void)
{
    return s_armed;
}

static void App_RC_MotorTestTask(void)
{
    rc_pwm_data_t rc;
    uint16_t throttle;
    uint16_t arm_ch;

    if (!s_motor_test_enable || !s_pwm_inited) {
        return;
    }

    RC_PWM_GetAll(&rc);

    throttle = rc.ch[RC_THROTTLE_CH - 1];
    arm_ch   = rc.ch[RC_ARM_CH - 1];

    /*
     * 接收机信号无效，立即锁定停机
     */
    if (!rc.valid[RC_THROTTLE_CH - 1] || !rc.valid[RC_ARM_CH - 1]) {
        s_armed = 0;
        s_motor_out = MOTOR_MIN_US;
        PWM_StopAll();
        return;
    }

    /*
     * CH5 低位：锁定
     */
    if (arm_ch < RC_ARM_OFF_US) {
        s_armed = 0;
        s_motor_out = MOTOR_MIN_US;
        PWM_StopAll();
        return;
    }

    /*
     * CH5 高位 + 油门最低：允许解锁
     */
    if (arm_ch > RC_ARM_ON_US) {
        if (!s_armed) {
            if (throttle < RC_THROTTLE_LOW_US) {
                s_armed = 1;
            }
        }
    } else {
        /*
         * CH5 在中间位置，不允许电机转
         */
        s_armed = 0;
        s_motor_out = MOTOR_MIN_US;
        PWM_StopAll();
        return;
    }

    /*
     * 未解锁：停机
     */
    if (!s_armed) {
        s_motor_out = MOTOR_MIN_US;
        PWM_StopAll();
        return;
    }

    /*
     * 已解锁，但油门最低：保持停机
     */
    if (throttle < RC_THROTTLE_LOW_US) {
        s_motor_out = MOTOR_MIN_US;
        PWM_StopAll();
        return;
    }

    /*
     * 已解锁 + 推油门：
     * 四个电机一起升降
     */
    s_motor_out = map_throttle_to_motor(throttle);

    PWM_SetAllMotorUs(s_motor_out,
                      s_motor_out,
                      s_motor_out,
                      s_motor_out);
}

static void App_RC_DebugPrintTask(void)
{
    rc_pwm_data_t rc;
    char buf[200];

    if (!s_rc_debug_print) {
        return;
    }

    RC_PWM_GetAll(&rc);

    snprintf(buf, sizeof(buf),
             "RC %s | CH1:%4u%c CH2:%4u%c CH3:%4u%c CH4:%4u%c CH5:%4u%c | ARM:%u OUT:%4u\r\n",
             RC_PWM_IsAllValid() ? "OK" : "NO",
             rc.ch[0], rc.valid[0] ? ' ' : '!',
             rc.ch[1], rc.valid[1] ? ' ' : '!',
             rc.ch[2], rc.valid[2] ? ' ' : '!',
             rc.ch[3], rc.valid[3] ? ' ' : '!',
             rc.ch[4], rc.valid[4] ? ' ' : '!',
             s_armed,
             s_motor_out);

    UART1_SendData_NonBlocking((uint8_t *)buf, strlen(buf));
}
