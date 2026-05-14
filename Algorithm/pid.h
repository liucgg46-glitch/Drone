#ifndef __PID_H
#define __PID_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float kp;
    float ki;
    float kd;

    float integrator;
    float prev_error;
    float prev_measurement;
    float d_term;

    float output_min;
    float output_max;
    float integrator_min;
    float integrator_max;

    float d_lpf_alpha;     /* 0..1, 1 means no filtering */
    unsigned char first_run;
} pid_t;

void PID_Init(pid_t *p, float kp, float ki, float kd,
              float out_min, float out_max,
              float i_min, float i_max,
              float d_lpf_alpha);
void PID_Reset(pid_t *p);
float PID_Update(pid_t *p, float setpoint, float measurement, float dt);
float PID_UpdateRate(pid_t *p, float setpoint_rate, float measured_rate, float dt);

#ifdef __cplusplus
}
#endif

#endif
