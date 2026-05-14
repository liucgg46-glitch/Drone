#include "lowpass.h"

#define LPF_PI 3.14159265358979323846f

void LowPass1_SetAlpha(lowpass1_t *f, float alpha)
{
    if (!f) return;
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;
    f->alpha = alpha;
}

void LowPass1_Init(lowpass1_t *f, float cutoff_hz, float dt)
{
    float rc;
    float alpha;
    if (!f) return;
    if (cutoff_hz <= 0.0f || dt <= 0.0f) {
        alpha = 1.0f;
    } else {
        rc = 1.0f / (2.0f * LPF_PI * cutoff_hz);
        alpha = dt / (rc + dt);
    }
    f->alpha = alpha;
    f->y = 0.0f;
    f->initialized = 0;
}

float LowPass1_Update(lowpass1_t *f, float x)
{
    if (!f) return x;
    if (!f->initialized) {
        f->y = x;
        f->initialized = 1;
        return x;
    }
    f->y += f->alpha * (x - f->y);
    return f->y;
}

void LowPass1_Reset(lowpass1_t *f, float value)
{
    if (!f) return;
    f->y = value;
    f->initialized = 1;
}
