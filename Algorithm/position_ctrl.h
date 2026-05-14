#ifndef __POSITION_CTRL_H
#define __POSITION_CTRL_H

#include "pid.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float target_x_m;
    float target_y_m;
    float measured_x_m;
    float measured_y_m;
    float vision_x_error;  /* normalized, right positive */
    float vision_y_error;  /* normalized, forward positive */
    unsigned char use_vision_error;
} position_input_t;

typedef struct {
    float target_roll_deg;
    float target_pitch_deg;
} position_output_t;

typedef struct {
    pid_t x_pid;
    pid_t y_pid;
    float max_angle_deg;
    float vision_gain_deg;
} position_ctrl_t;

void PositionCtrl_Init(position_ctrl_t *c);
void PositionCtrl_Reset(position_ctrl_t *c);
position_output_t PositionCtrl_Update(position_ctrl_t *c, const position_input_t *in, float dt);

#ifdef __cplusplus
}
#endif

#endif
