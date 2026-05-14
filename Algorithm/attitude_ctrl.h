#ifndef __ATTITUDE_CTRL_H
#define __ATTITUDE_CTRL_H

#include "pid.h"
#include "quaternion.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float roll_deg;
    float pitch_deg;
    float yaw_rate_deg_s;
    float throttle;        /* 1000..2000 us or normalized by caller */
} attitude_sp_t;

typedef struct {
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    float gyro_x_deg_s;
    float gyro_y_deg_s;
    float gyro_z_deg_s;
} attitude_meas_t;

typedef struct {
    float roll_cmd;
    float pitch_cmd;
    float yaw_cmd;
    float motor[4];        /* M1 M2 M3 M4 mixed output, same unit as throttle */
} attitude_ctrl_out_t;

typedef struct {
    pid_t angle_roll;
    pid_t angle_pitch;
    pid_t rate_roll;
    pid_t rate_pitch;
    pid_t rate_yaw;
    float max_angle_rate_deg_s;
    float max_correction;
} attitude_ctrl_t;

void AttitudeCtrl_Init(attitude_ctrl_t *c);
void AttitudeCtrl_Reset(attitude_ctrl_t *c);
attitude_ctrl_out_t AttitudeCtrl_Update(attitude_ctrl_t *c,
                                         const attitude_sp_t *sp,
                                         const attitude_meas_t *meas,
                                         float dt);

#ifdef __cplusplus
}
#endif

#endif
