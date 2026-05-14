#include "attitude_ctrl.h"

static float clampf(float x, float min_v, float max_v)
{
    if (x < min_v) return min_v;
    if (x > max_v) return max_v;
    return x;
}

void AttitudeCtrl_Init(attitude_ctrl_t *c)
{
    if (!c) return;

    /* Conservative default values for simulation. Must be tuned on real frame. */
    PID_Init(&c->angle_roll,  4.0f, 0.0f, 0.0f, -180.0f, 180.0f, -50.0f, 50.0f, 1.0f);
    PID_Init(&c->angle_pitch, 4.0f, 0.0f, 0.0f, -180.0f, 180.0f, -50.0f, 50.0f, 1.0f);

    PID_Init(&c->rate_roll,   0.08f, 0.02f, 0.001f, -250.0f, 250.0f, -80.0f, 80.0f, 0.25f);
    PID_Init(&c->rate_pitch,  0.08f, 0.02f, 0.001f, -250.0f, 250.0f, -80.0f, 80.0f, 0.25f);
    PID_Init(&c->rate_yaw,    0.12f, 0.01f, 0.000f, -250.0f, 250.0f, -80.0f, 80.0f, 0.25f);

    c->max_angle_rate_deg_s = 180.0f;
    c->max_correction = 250.0f;
}

void AttitudeCtrl_Reset(attitude_ctrl_t *c)
{
    if (!c) return;
    PID_Reset(&c->angle_roll);
    PID_Reset(&c->angle_pitch);
    PID_Reset(&c->rate_roll);
    PID_Reset(&c->rate_pitch);
    PID_Reset(&c->rate_yaw);
}

attitude_ctrl_out_t AttitudeCtrl_Update(attitude_ctrl_t *c,
                                         const attitude_sp_t *sp,
                                         const attitude_meas_t *meas,
                                         float dt)
{
    attitude_ctrl_out_t o;
    float target_roll_rate;
    float target_pitch_rate;
    float r, p, y, t;

    o.roll_cmd = o.pitch_cmd = o.yaw_cmd = 0.0f;
    o.motor[0] = o.motor[1] = o.motor[2] = o.motor[3] = 0.0f;

    if (!c || !sp || !meas || dt <= 0.0f) return o;

    target_roll_rate = PID_Update(&c->angle_roll, sp->roll_deg, meas->roll_deg, dt);
    target_pitch_rate = PID_Update(&c->angle_pitch, sp->pitch_deg, meas->pitch_deg, dt);
    target_roll_rate = clampf(target_roll_rate, -c->max_angle_rate_deg_s, c->max_angle_rate_deg_s);
    target_pitch_rate = clampf(target_pitch_rate, -c->max_angle_rate_deg_s, c->max_angle_rate_deg_s);

    o.roll_cmd = PID_UpdateRate(&c->rate_roll, target_roll_rate, meas->gyro_x_deg_s, dt);
    o.pitch_cmd = PID_UpdateRate(&c->rate_pitch, target_pitch_rate, meas->gyro_y_deg_s, dt);
    o.yaw_cmd = PID_UpdateRate(&c->rate_yaw, sp->yaw_rate_deg_s, meas->gyro_z_deg_s, dt);

    o.roll_cmd = clampf(o.roll_cmd, -c->max_correction, c->max_correction);
    o.pitch_cmd = clampf(o.pitch_cmd, -c->max_correction, c->max_correction);
    o.yaw_cmd = clampf(o.yaw_cmd, -c->max_correction, c->max_correction);

    /* X quad mixer: M1 front-left, M2 front-right, M3 rear-right, M4 rear-left.
       Adjust signs after checking real motor layout. */
    t = sp->throttle;
    r = o.roll_cmd;
    p = o.pitch_cmd;
    y = o.yaw_cmd;

    o.motor[0] = t + p + r - y;
    o.motor[1] = t + p - r + y;
    o.motor[2] = t - p - r - y;
    o.motor[3] = t - p + r + y;

    return o;
}
