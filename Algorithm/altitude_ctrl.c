#include "altitude_ctrl.h"

static float limitf(float x, float min_v, float max_v)
{
    if (x < min_v) return min_v;
    if (x > max_v) return max_v;
    return x;
}

void AltitudeCtrl_Init(altitude_ctrl_t *c, float dt)
{
    if (!c) return;
    PID_Init(&c->height_pid, 2.0f, 0.0f, 0.0f, -0.8f, 0.8f, -0.2f, 0.2f, 1.0f);
    PID_Init(&c->vz_pid,     90.0f, 20.0f, 0.0f, -250.0f, 250.0f, -120.0f, 120.0f, 0.3f);
    LowPass1_Init(&c->height_lpf, 8.0f, dt);
    c->hover_throttle = 1400.0f;
    c->min_throttle = 1000.0f;
    c->max_throttle = 1800.0f;
    c->max_vz_m_s = 0.8f;
}

void AltitudeCtrl_Reset(altitude_ctrl_t *c)
{
    if (!c) return;
    PID_Reset(&c->height_pid);
    PID_Reset(&c->vz_pid);
}

float AltitudeCtrl_Update(altitude_ctrl_t *c, const altitude_input_t *in, float dt)
{
    float h;
    float target_vz;
    float throttle_delta;

    if (!c || !in || dt <= 0.0f) return 1000.0f;

    h = LowPass1_Update(&c->height_lpf, in->measured_height_m);
    target_vz = PID_Update(&c->height_pid, in->target_height_m, h, dt);
    target_vz = limitf(target_vz, -c->max_vz_m_s, c->max_vz_m_s);

    throttle_delta = PID_Update(&c->vz_pid, target_vz, in->vertical_speed_m_s, dt);
    return limitf(c->hover_throttle + throttle_delta, c->min_throttle, c->max_throttle);
}
