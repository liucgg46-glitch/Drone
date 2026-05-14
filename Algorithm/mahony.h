#ifndef __MAHONY_H
#define __MAHONY_H

#include <stdint.h>

void Mahony_Init(float sample_freq);
void Mahony_Update(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
void Mahony_GetEuler(float *roll, float *pitch, float *yaw);

#endif
