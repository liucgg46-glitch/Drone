#ifndef __LOWPASS_H
#define __LOWPASS_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float alpha;
    float y;
    unsigned char initialized;
} lowpass1_t;

void LowPass1_Init(lowpass1_t *f, float cutoff_hz, float dt);
void LowPass1_SetAlpha(lowpass1_t *f, float alpha);
float LowPass1_Update(lowpass1_t *f, float x);
void LowPass1_Reset(lowpass1_t *f, float value);

#ifdef __cplusplus
}
#endif

#endif
