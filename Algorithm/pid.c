#include "pid.h"

static float constrainf_local(float x, float min_v, float max_v)
{
    if (x < min_v) return min_v;
    if (x > max_v) return max_v;
    return x;
}//把 x 限制在 min_v 到 max_v 之间

void PID_Init(pid_t *p, float kp, float ki, float kd,
              float out_min, float out_max,
              float i_min, float i_max,
              float d_lpf_alpha)
{
    if (!p) return;
    p->kp = kp;
    p->ki = ki;
    p->kd = kd;
    p->output_min = out_min;
    p->output_max = out_max;
    p->integrator_min = i_min;
    p->integrator_max = i_max;
    if (d_lpf_alpha < 0.0f) d_lpf_alpha = 0.0f;
    if (d_lpf_alpha > 1.0f) d_lpf_alpha = 1.0f;
    p->d_lpf_alpha = d_lpf_alpha;
    PID_Reset(p);
}

void PID_Reset(pid_t *p)
{
    if (!p) return;
    p->integrator = 0.0f;
    p->prev_error = 0.0f;
    p->prev_measurement = 0.0f;
    p->d_term = 0.0f;
    p->first_run = 1;
}

float PID_Update(pid_t *p, float setpoint, float measurement, float dt)//控制器，目标值，当前测量值，控制周期
{
    float error;
    float derivative;
    float output;
    if (!p || dt <= 0.0f) return 0.0f;

    error = setpoint - measurement;

    p->integrator += error * p->ki * dt;
    p->integrator = constrainf_local(p->integrator, p->integrator_min, p->integrator_max);

    /* Derivative on measurement reduces setpoint kick. */
    if (p->first_run) {
        derivative = 0.0f;
        p->first_run = 0;
    } else {
        derivative = -(measurement - p->prev_measurement) / dt;
    }
    p->d_term += p->d_lpf_alpha * (derivative - p->d_term);

    output = p->kp * error + p->integrator + p->kd * p->d_term;
    output = constrainf_local(output, p->output_min, p->output_max);

    p->prev_error = error;
    p->prev_measurement = measurement;
    return output;
}

float PID_UpdateRate(pid_t *p, float setpoint_rate, float measured_rate, float dt)
{
    /* Same as normal PID but variable names fit rate loop. */
    return PID_Update(p, setpoint_rate, measured_rate, dt);
}
