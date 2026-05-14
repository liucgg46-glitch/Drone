#include "ahrs_fusion.h"

void AhrsFusion_Init(ahrs_fusion_t *f, float dt)
{
    if (!f) return;
    Mahony_Init(&f->mahony, 2.0f, 0.05f);
    LowPass1_Init(&f->height_lpf, 8.0f, dt);
    f->last_height_m = 0.0f;
    f->height_initialized = 0;
}

fusion_output_t AhrsFusion_Update(ahrs_fusion_t *f, const fusion_input_t *in, float dt)
{
    fusion_output_t out;
    out.attitude_deg.roll = 0.0f;
    out.attitude_deg.pitch = 0.0f;
    out.attitude_deg.yaw = 0.0f;
    quat_set_identity(&out.q);
    out.height_m = 0.0f;
    out.vertical_speed_m_s = 0.0f;

    if (!f || !in || dt <= 0.0f) return out;

    if (in->imu_valid) {
        Mahony_UpdateIMU(&f->mahony,
                         in->gx_rad_s, in->gy_rad_s, in->gz_rad_s,
                         in->ax_g, in->ay_g, in->az_g,
                         dt);
    }

    out.attitude_deg = f->mahony.euler;
    out.q = f->mahony.q;

    if (in->height_valid) {
        out.height_m = LowPass1_Update(&f->height_lpf, in->height_m);
        if (!f->height_initialized) {
            f->last_height_m = out.height_m;
            f->height_initialized = 1;
        }
        out.vertical_speed_m_s = (out.height_m - f->last_height_m) / dt;
        f->last_height_m = out.height_m;
    } else {
        out.height_m = f->last_height_m;
        out.vertical_speed_m_s = 0.0f;
    }

    return out;
}
