#ifndef __AHRS_FUSION_H
#define __AHRS_FUSION_H

#include "mahony.h"
#include "lowpass.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float ax_g, ay_g, az_g;
    float gx_rad_s, gy_rad_s, gz_rad_s;
    float height_m;
    unsigned char imu_valid;
    unsigned char height_valid;
} fusion_input_t;

typedef struct {
    euler_t attitude_deg;
    quat_t q;
    float height_m;
    float vertical_speed_m_s;
} fusion_output_t;

typedef struct {
    mahony_t mahony;
    lowpass1_t height_lpf;
    float last_height_m;
    unsigned char height_initialized;
} ahrs_fusion_t;

void AhrsFusion_Init(ahrs_fusion_t *f, float dt);
fusion_output_t AhrsFusion_Update(ahrs_fusion_t *f, const fusion_input_t *in, float dt);

#ifdef __cplusplus
}
#endif

#endif
