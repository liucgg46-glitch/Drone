#include "position_ctrl.h"

static float clipf(float x, float min_v, float max_v)
{
    if (x < min_v) return min_v;
    if (x > max_v) return max_v;
    return x;
}

void PositionCtrl_Init(position_ctrl_t *c)
{
    if (!c) return;
    PID_Init(&c->x_pid, 1.2f, 0.0f, 0.10f, -10.0f, 10.0f, -3.0f, 3.0f, 0.3f);
    PID_Init(&c->y_pid, 1.2f, 0.0f, 0.10f, -10.0f, 10.0f, -3.0f, 3.0f, 0.3f);
    c->max_angle_deg = 10.0f;
    c->vision_gain_deg = 8.0f;
}

void PositionCtrl_Reset(position_ctrl_t *c)
{
    if (!c) return;
    PID_Reset(&c->x_pid);
    PID_Reset(&c->y_pid);
}

position_output_t PositionCtrl_Update(position_ctrl_t *c, const position_input_t *in, float dt)
{
    position_output_t o;
    o.target_roll_deg = 0.0f;
    o.target_pitch_deg = 0.0f;

    if (!c || !in || dt <= 0.0f) return o;

    if (in->use_vision_error) {
        /* Vision line/QR center error directly converts to small attitude angle. */
        o.target_roll_deg  =  c->vision_gain_deg * in->vision_x_error;
        o.target_pitch_deg = -c->vision_gain_deg * in->vision_y_error;
    } else {
        /* Optical-flow estimated x/y position holding. */
        o.target_roll_deg  =  PID_Update(&c->x_pid, in->target_x_m, in->measured_x_m, dt);
        o.target_pitch_deg = -PID_Update(&c->y_pid, in->target_y_m, in->measured_y_m, dt);
    }

    o.target_roll_deg = clipf(o.target_roll_deg, -c->max_angle_deg, c->max_angle_deg);
    o.target_pitch_deg = clipf(o.target_pitch_deg, -c->max_angle_deg, c->max_angle_deg);
    return o;
}
