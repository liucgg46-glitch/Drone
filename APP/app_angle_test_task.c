#include "app_angle_test_task.h"

#include "task_scheduler.h"
#include "app_attitude_task.h"
#include "attitude_ctrl.h"
#include "bsp_uart.h"
#include "bsp_pwm.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

/*
 * 角度外环测试：
 *
 * 目标：
 * roll  = 0 deg
 * pitch = 0 deg
 * yaw_rate = 0 deg/s
 *
 * throttle 先用 1100us，无桨测试。
 */

#define ANGLE_TEST_DT_S              0.005f
#define ANGLE_TEST_BASE_THROTTLE     1100.0f

#define ANGLE_TEST_PWM_MIN_US        1000.0f
#define ANGLE_TEST_PWM_MAX_US        1200.0f

static attitude_ctrl_t s_ctrl;

static uint8_t s_inited = 0;
static uint8_t s_debug_print = 1;
static uint8_t s_pwm_output_enable = 0;

static attitude_sp_t s_sp;
static attitude_meas_t s_meas;
static attitude_ctrl_out_t s_out;

static void App_AngleTest_UpdateTask(void);
static void App_AngleTest_PrintTask(void);

static task_t s_task_angle_update = {
    .func = App_AngleTest_UpdateTask,
    .interval_ms = 5,
    .last_run = 0
};

static task_t s_task_angle_print = {
    .func = App_AngleTest_PrintTask,
    .interval_ms = 50,
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
    x = clampf_local(x, ANGLE_TEST_PWM_MIN_US, ANGLE_TEST_PWM_MAX_US);
    return (uint16_t)(x + 0.5f);
}

void App_AngleTest_Init(void)
{
    AttitudeCtrl_Init(&s_ctrl);

    /*
     * 角度外环目标：
     * 先让飞机尝试保持水平。
     */
    s_sp.roll_deg = 0.0f;
    s_sp.pitch_deg = 0.0f;
    s_sp.yaw_rate_deg_s = 0.0f;
    s_sp.throttle = ANGLE_TEST_BASE_THROTTLE;

    memset(&s_meas, 0, sizeof(s_meas));
    memset(&s_out, 0, sizeof(s_out));

    s_pwm_output_enable = 0;

    PWM_SetAllMotorUs(1000, 1000, 1000, 1000);

    s_inited = 1;
}

void App_AngleTest_RegisterTasks(void)
{
    scheduler_register(&s_task_angle_update);
    scheduler_register(&s_task_angle_print);
}

void App_AngleTest_SetDebugPrint(uint8_t enable)
{
    s_debug_print = enable ? 1U : 0U;
}

void App_AngleTest_EnablePwmOutput(uint8_t enable)
{
    s_pwm_output_enable = enable ? 1U : 0U;

    if (!s_pwm_output_enable) {
        PWM_SetAllMotorUs(1000, 1000, 1000, 1000);
    }
}

static void App_AngleTest_UpdateTask(void)
{
    const app_attitude_data_t *att;
    float m1;
    float m2;
    float m3;
    float m4;

    if (!s_inited) {
        return;
    }

    att = App_Attitude_GetData();

    if (att == 0 || !att->valid) {
        if (s_pwm_output_enable) {
            PWM_SetAllMotorUs(1000, 1000, 1000, 1000);
        }
        return;
    }

    /*
     * 姿态测量量
     */
    s_meas.roll_deg = att->roll_deg;
    s_meas.pitch_deg = att->pitch_deg;
    s_meas.yaw_deg = att->yaw_deg;

    s_meas.gyro_x_deg_s = att->gx_dps;
    s_meas.gyro_y_deg_s = att->gy_dps;
    s_meas.gyro_z_deg_s = att->gz_dps;

    /*
     * 串级控制：
     * 角度环 -> 角速度环 -> 混控
     */
    s_out = AttitudeCtrl_Update(&s_ctrl, &s_sp, &s_meas, ANGLE_TEST_DT_S);

    /*
     * 无桨测试阶段限幅到 1000~1200us。
     */
    m1 = clampf_local(s_out.motor[0], ANGLE_TEST_PWM_MIN_US, ANGLE_TEST_PWM_MAX_US);
    m2 = clampf_local(s_out.motor[1], ANGLE_TEST_PWM_MIN_US, ANGLE_TEST_PWM_MAX_US);
    m3 = clampf_local(s_out.motor[2], ANGLE_TEST_PWM_MIN_US, ANGLE_TEST_PWM_MAX_US);
    m4 = clampf_local(s_out.motor[3], ANGLE_TEST_PWM_MIN_US, ANGLE_TEST_PWM_MAX_US);

    s_out.motor[0] = m1;
    s_out.motor[1] = m2;
    s_out.motor[2] = m3;
    s_out.motor[3] = m4;

    if (s_pwm_output_enable) {
        PWM_SetAllMotorUs(float_to_pwm_us(m1),
                          float_to_pwm_us(m2),
                          float_to_pwm_us(m3),
                          float_to_pwm_us(m4));
    }
}

static void App_AngleTest_PrintTask(void)
{
    char buf[260];

    if (!s_inited || !s_debug_print) {
        return;
    }

    snprintf(buf,
             sizeof(buf),
             "ANGLE R:% .2f P:% .2f | CMD R:% .1f P:% .1f Y:% .1f | M:% .1f % .1f % .1f % .1f | PWM:%u\r\n",
             s_meas.roll_deg,
             s_meas.pitch_deg,
             s_out.roll_cmd,
             s_out.pitch_cmd,
             s_out.yaw_cmd,
             s_out.motor[0],
             s_out.motor[1],
             s_out.motor[2],
             s_out.motor[3],
             (unsigned)s_pwm_output_enable);

    UART1_SendData_NonBlocking((uint8_t *)buf, (uint16_t)strlen(buf));
}