#include "app_rate_test_task.h"

#include "task_scheduler.h"
#include "app_attitude_task.h"
#include "pid.h"
#include "bsp_uart.h"
#include "bsp_pwm.h"

#include <stdio.h>
#include <string.h>
#include <stdint.h>

/*
 * 角速度环测试任务
 *
 * 目的：
 * 1. 只做角速度环，不做角度外环
 * 2. 目标角速度全部为 0
 * 3. 用 gyro_x/y/z 做 PID
 * 4. 混控得到 M1~M4
 * 5. 调试阶段可以选择是否真正输出 PWM
 *
 * 电机编号：
 *
 *        机头 / 前方
 *
 *      M1        M2
 *    左前      右前
 *
 *        飞控
 *
 *      M4        M3
 *    左后      右后
 *
 * PWM 对应：
 * M1 -> PE9  / TIM1_CH1
 * M2 -> PE11 / TIM1_CH2
 * M3 -> PE13 / TIM1_CH3
 * M4 -> PE14 / TIM1_CH4
 */

/*
 * 5ms 更新一次角速度环
 */
#define RATE_TEST_DT_S              0.005f

/*
 * 无桨电机测试基础油门
 * 先用 1100us，比较安全。
 */
#define RATE_TEST_BASE_THROTTLE     1100.0f

/*
 * 无桨测试阶段限制 PWM 范围
 * 不要一开始给太大。
 */
#define RATE_TEST_PWM_MIN_US        1000.0f
#define RATE_TEST_PWM_MAX_US        1200.0f

/*
 * PID 输出限幅
 * 先小一点，主要听电机变化方向。
 */
#define RATE_TEST_CMD_MIN           -80.0f
#define RATE_TEST_CMD_MAX            80.0f

static pid_t s_rate_roll;
static pid_t s_rate_pitch;
static pid_t s_rate_yaw;

static uint8_t s_inited = 0;
static uint8_t s_debug_print = 1;

/*
 * 默认禁止真实 PWM 输出。
 * main.c 里调用 App_RateTest_EnablePwmOutput(1) 后才允许输出。
 */
static uint8_t s_pwm_output_enable = 0;

static float s_roll_cmd = 0.0f;
static float s_pitch_cmd = 0.0f;
static float s_yaw_cmd = 0.0f;

static float s_m1 = RATE_TEST_BASE_THROTTLE;
static float s_m2 = RATE_TEST_BASE_THROTTLE;
static float s_m3 = RATE_TEST_BASE_THROTTLE;
static float s_m4 = RATE_TEST_BASE_THROTTLE;

static void App_RateTest_UpdateTask(void);
static void App_RateTest_PrintTask(void);

static task_t s_task_rate_update = {
    .func = App_RateTest_UpdateTask,
    .interval_ms = 5,
    .last_run = 0
};

/*
 * 20ms 打印一次，更容易抓到手动转动瞬间
 */
static task_t s_task_rate_print = {
    .func = App_RateTest_PrintTask,
    .interval_ms = 20,
    .last_run = 0
};

static float clampf_local(float x, float min_v, float max_v)
{
    if (x < min_v) {
        return min_v;
    }

    if (x > max_v) {
        return max_v;
    }

    return x;
}

static uint16_t float_to_pwm_us(float x)
{
    x = clampf_local(x, RATE_TEST_PWM_MIN_US, RATE_TEST_PWM_MAX_US);

    return (uint16_t)(x + 0.5f);
}

void App_RateTest_Init(void)
{
    /*
     * 先只用 P，不开 I，不开 D。
     * 这样最容易判断电机变化方向。
     *
     * PID_UpdateRate 内部是：
     * error = setpoint_rate - measured_rate
     *
     * 所以目标角速度为 0 时：
     * 当前 gx 为正，输出会为负；
     * 当前 gx 为负，输出会为正。
     */
    PID_Init(&s_rate_roll,
             0.8f, 0.0f, 0.0f,
             RATE_TEST_CMD_MIN, RATE_TEST_CMD_MAX,
             -30.0f, 30.0f,
             0.2f);

    PID_Init(&s_rate_pitch,
             0.8f, 0.0f, 0.0f,
             RATE_TEST_CMD_MIN, RATE_TEST_CMD_MAX,
             -30.0f, 30.0f,
             0.2f);

    PID_Init(&s_rate_yaw,
             0.8f, 0.0f, 0.0f,
             RATE_TEST_CMD_MIN, RATE_TEST_CMD_MAX,
             -30.0f, 30.0f,
             0.2f);

    s_roll_cmd = 0.0f;
    s_pitch_cmd = 0.0f;
    s_yaw_cmd = 0.0f;

    s_m1 = RATE_TEST_BASE_THROTTLE;
    s_m2 = RATE_TEST_BASE_THROTTLE;
    s_m3 = RATE_TEST_BASE_THROTTLE;
    s_m4 = RATE_TEST_BASE_THROTTLE;

    s_pwm_output_enable = 0;

    /*
     * 初始化后先强制所有电机 1000us
     */
    PWM_SetAllMotorUs(1000, 1000, 1000, 1000);

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

void App_RateTest_EnablePwmOutput(uint8_t enable)
{
    s_pwm_output_enable = enable ? 1U : 0U;

    if (!s_pwm_output_enable) {
        PWM_SetAllMotorUs(1000, 1000, 1000, 1000);
    }
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
        /*
         * 姿态还没准备好时，保持安全输出
         */
        if (s_pwm_output_enable) {
            PWM_SetAllMotorUs(1000, 1000, 1000, 1000);
        }
        return;
    }

    /*
     * 只做角速度环：
     * 目标角速度全部为 0。
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

    s_roll_cmd = clampf_local(s_roll_cmd, RATE_TEST_CMD_MIN, RATE_TEST_CMD_MAX);
    s_pitch_cmd = clampf_local(s_pitch_cmd, RATE_TEST_CMD_MIN, RATE_TEST_CMD_MAX);
    s_yaw_cmd = clampf_local(s_yaw_cmd, RATE_TEST_CMD_MIN, RATE_TEST_CMD_MAX);

    /*
     * X 型四轴混控
     *
     * M1 左前
     * M2 右前
     * M3 右后
     * M4 左后
     *
     * 你前面测试过：
     * 右偏时 CMD R 为负，应该 M2/M3 变大；
     * 低头时 CMD P 为正，应该 M1/M2 变大。
     */
    t = RATE_TEST_BASE_THROTTLE;
    r = s_roll_cmd;
    p = s_pitch_cmd;
    y = s_yaw_cmd;

    s_m1 = t + p + r - y;
    s_m2 = t + p - r + y;
    s_m3 = t - p - r - y;
    s_m4 = t - p + r + y;

    s_m1 = clampf_local(s_m1, RATE_TEST_PWM_MIN_US, RATE_TEST_PWM_MAX_US);
    s_m2 = clampf_local(s_m2, RATE_TEST_PWM_MIN_US, RATE_TEST_PWM_MAX_US);
    s_m3 = clampf_local(s_m3, RATE_TEST_PWM_MIN_US, RATE_TEST_PWM_MAX_US);
    s_m4 = clampf_local(s_m4, RATE_TEST_PWM_MIN_US, RATE_TEST_PWM_MAX_US);

    if (s_pwm_output_enable) {
        PWM_SetAllMotorUs(float_to_pwm_us(s_m1),
                          float_to_pwm_us(s_m2),
                          float_to_pwm_us(s_m3),
                          float_to_pwm_us(s_m4));
    }
}

static void App_RateTest_PrintTask(void)
{
    const app_attitude_data_t *att;
    char buf[260];

    if (!s_inited || !s_debug_print) {
        return;
    }

    att = App_Attitude_GetData();

    if (att == 0 || !att->valid) {
        UART1_SendData_NonBlocking((uint8_t *)"RATE waiting attitude\r\n",
                                   (uint16_t)strlen("RATE waiting attitude\r\n"));
        return;
    }

    snprintf(buf,
             sizeof(buf),
             "RATE G:% .2f % .2f % .2f | CMD R:% .1f P:% .1f Y:% .1f | M:% .1f % .1f % .1f % .1f | PWM:%u\r\n",
             att->gx_dps,
             att->gy_dps,
             att->gz_dps,
             s_roll_cmd,
             s_pitch_cmd,
             s_yaw_cmd,
             s_m1,
             s_m2,
             s_m3,
             s_m4,
             (unsigned)s_pwm_output_enable);

    UART1_SendData_NonBlocking((uint8_t *)buf, (uint16_t)strlen(buf));
}