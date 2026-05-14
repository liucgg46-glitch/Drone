#include "app_rate_test_task.h"

#include "task_scheduler.h"
#include "app_attitude_task.h"
#include "pid.h"
#include "bsp_uart.h"

#include <stdio.h>
#include <string.h>
#include <stdint.h>

/*
 * 角速度环测试任务
 *
 * 目的：
 * 1. 不接 PWM
 * 2. 不让电机转
 * 3. 只打印 roll_rate / pitch_rate / yaw_rate 的 PID 输出
 * 4. 只打印四个电机混控值
 */

#define RATE_TEST_DT_S          0.005f
#define RATE_TEST_BASE_THROTTLE 1200.0f

static pid_t s_rate_roll;
static pid_t s_rate_pitch;
static pid_t s_rate_yaw;

static uint8_t s_inited = 0;
static uint8_t s_debug_print = 1;

static float s_roll_cmd = 0.0f;
static float s_pitch_cmd = 0.0f;
static float s_yaw_cmd = 0.0f;

static float s_m1 = 0.0f;
static float s_m2 = 0.0f;
static float s_m3 = 0.0f;
static float s_m4 = 0.0f;

static void App_RateTest_UpdateTask(void);
static void App_RateTest_PrintTask(void);

static task_t s_task_rate_update = {
    .func = App_RateTest_UpdateTask,
    .interval_ms = 5,
    .last_run = 0
};

static task_t s_task_rate_print = {
    .func = App_RateTest_PrintTask,
    .interval_ms = 20,
    .last_run = 0
};

static float clampf_local(float x, float min_v, float max_v)
{
    if (x < min_v) return min_v;
    if (x > max_v) return max_v;
    return x;
}

void App_RateTest_Init(void)
{
    /*
     * 先只用 P，不开 I，不开 D。
     * 这样最容易判断方向。
     *
     * 输出限幅先小一点，避免后面接电机时太猛。
     */
    PID_Init(&s_rate_roll,
             0.8f, 0.0f, 0.0f,
             -200.0f, 200.0f,
             -50.0f, 50.0f,
             0.2f);

    PID_Init(&s_rate_pitch,
             0.8f, 0.0f, 0.0f,
             -200.0f, 200.0f,
             -50.0f, 50.0f,
             0.2f);

    PID_Init(&s_rate_yaw,
             0.8f, 0.0f, 0.0f,
             -200.0f, 200.0f,
             -50.0f, 50.0f,
             0.2f);

    s_roll_cmd = 0.0f;
    s_pitch_cmd = 0.0f;
    s_yaw_cmd = 0.0f;

    s_m1 = RATE_TEST_BASE_THROTTLE;
    s_m2 = RATE_TEST_BASE_THROTTLE;
    s_m3 = RATE_TEST_BASE_THROTTLE;
    s_m4 = RATE_TEST_BASE_THROTTLE;

    s_inited = 1;
}

void App_RateTest_RegisterTasks(void)
{
    scheduler_register(&s_task_rate_update);
    scheduler_register(&s_task_rate_print);
}

void App_RateTest_SetDebugPrint(uint8_t enable)
{
    s_debug_print = enable ? 1U : 0U;
}

static void App_RateTest_UpdateTask(void)
{
    const app_attitude_data_t *att;
    float t;
    float r;
    float p;
    float y;

    if (!s_inited) {
        return;
    }

    att = App_Attitude_GetData();

    if (att == 0 || !att->valid) {
        return;
    }

    /*
     * 只做角速度环：
     * 目标角速度全部为 0。
     *
     * 注意：
     * PID_UpdateRate 内部是 error = setpoint - measurement。
     * 所以当前向右滚转 gyro_x 为正时，输出会是负值，
     * 作用是产生反向修正。
     */
    s_roll_cmd = PID_UpdateRate(&s_rate_roll,
                                0.0f,
                                att->gx_dps,
                                RATE_TEST_DT_S);

    s_pitch_cmd = PID_UpdateRate(&s_rate_pitch,
                                 0.0f,
                                 att->gy_dps,
                                 RATE_TEST_DT_S);

    s_yaw_cmd = PID_UpdateRate(&s_rate_yaw,
                               0.0f,
                               att->gz_dps,
                               RATE_TEST_DT_S);

    s_roll_cmd = clampf_local(s_roll_cmd, -200.0f, 200.0f);
    s_pitch_cmd = clampf_local(s_pitch_cmd, -200.0f, 200.0f);
    s_yaw_cmd = clampf_local(s_yaw_cmd, -200.0f, 200.0f);

    /*
     * X 型四轴混控，先只打印，不输出 PWM。
     *
     * 假设：
     * M1 左前
     * M2 右前
     * M3 右后
     * M4 左后
     *
     * 后面真实接电机时，必须再按你的电机编号检查。
     */
    t = RATE_TEST_BASE_THROTTLE;
    r = s_roll_cmd;
    p = s_pitch_cmd;
    y = s_yaw_cmd;

    s_m1 = t + p + r - y;
    s_m2 = t + p - r + y;
    s_m3 = t - p - r - y;
    s_m4 = t - p + r + y;
}

static void App_RateTest_PrintTask(void)
{
    const app_attitude_data_t *att;
    char buf[240];

    if (!s_inited || !s_debug_print) {
        return;
    }

    att = App_Attitude_GetData();

    if (att == 0 || !att->valid) {
        UART1_SendData_NonBlocking((uint8_t *)"RATE waiting attitude\r\n",
                                   23);
        return;
    }

    snprintf(buf,
             sizeof(buf),
             "RATE G:% .2f % .2f % .2f | CMD R:% .1f P:% .1f Y:% .1f | M:% .1f % .1f % .1f % .1f\r\n",
             att->gx_dps,
             att->gy_dps,
             att->gz_dps,
             s_roll_cmd,
             s_pitch_cmd,
             s_yaw_cmd,
             s_m1,
             s_m2,
             s_m3,
             s_m4);

    UART1_SendData_NonBlocking((uint8_t *)buf, (uint16_t)strlen(buf));
}