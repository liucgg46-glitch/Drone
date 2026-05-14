#ifndef __ALTITUDE_CTRL_H
#define __ALTITUDE_CTRL_H

#include "pid.h"
#include "lowpass.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float target_height_m;
    float measured_height_m;
    float vertical_speed_m_s;
} altitude_input_t;

typedef struct {
    pid_t height_pid;
    pid_t vz_pid;
    lowpass1_t height_lpf;
    float hover_throttle;
    float min_throttle;
    float max_throttle;
    float max_vz_m_s;
} altitude_ctrl_t;

void AltitudeCtrl_Init(altitude_ctrl_t *c, float dt);
void AltitudeCtrl_Reset(altitude_ctrl_t *c);
float AltitudeCtrl_Update(altitude_ctrl_t *c, const altitude_input_t *in, float dt);

#ifdef __cplusplus
}
#endif

#endif
