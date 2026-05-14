#include "mahony.h"
#include <math.h>

static float twoKp = 2.0f * 0.5f;  // 比例增益
static float twoKi = 2.0f * 0.0f;  // 积分增益，通常设为0
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
static float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;
static float sample_freq;

void Mahony_Init(float freq)
{
    sample_freq = freq;
    q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
    integralFBx = 0.0f; integralFBy = 0.0f; integralFBz = 0.0f;
}

void Mahony_Update(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
    float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    float hx, hy, bx, bz;
    float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
    float ex, ey, ez;
    float qa, qb, qc;

    // 加速度归一化
    recipNorm = sqrtf(ax*ax + ay*ay + az*az);
    if (recipNorm > 0.0f) {
        recipNorm = 1.0f / recipNorm;
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;
    }

    // 磁力计归一化
    recipNorm = sqrtf(mx*mx + my*my + mz*mz);
    if (recipNorm > 0.0f) {
        recipNorm = 1.0f / recipNorm;
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;
    }

    // 四元数乘积
    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;

    // 参考方向的重力向量
    halfvx = q1q3 - q0q2;
    halfvy = q0q1 + q2q3;
    halfvz = q0q0 - 0.5f + q3q3;

    // 参考方向的磁场向量
    hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
    hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
    bx = sqrtf(hx*hx + hy*hy);
    bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

    // 估计磁场方向
    halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
    halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
    halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

    // 误差计算
    ex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
    ey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
    ez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

    // 积分误差
    if (twoKi > 0.0f) {
        integralFBx += twoKi * ex * (1.0f / sample_freq);
        integralFBy += twoKi * ey * (1.0f / sample_freq);
        integralFBz += twoKi * ez * (1.0f / sample_freq);
        gx += integralFBx;
        gy += integralFBy;
        gz += integralFBz;
    } else {
        integralFBx = 0.0f;
        integralFBy = 0.0f;
        integralFBz = 0.0f;
    }

    // 应用比例反馈
    gx += twoKp * ex;
    gy += twoKp * ey;
    gz += twoKp * ez;

    // 积分四元数
    gx *= 0.5f * (1.0f / sample_freq);
    gy *= 0.5f * (1.0f / sample_freq);
    gz *= 0.5f * (1.0f / sample_freq);
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += ( qa * gx + qc * gz - q3 * gy);
    q2 += ( qa * gy - qb * gz + q3 * gx);
    q3 += ( qa * gz + qb * gy - qc * gx);

    // 归一化四元数
    recipNorm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    if (recipNorm > 0.0f) {
        recipNorm = 1.0f / recipNorm;
        q0 *= recipNorm;
        q1 *= recipNorm;
        q2 *= recipNorm;
        q3 *= recipNorm;
    }
}

void Mahony_GetEuler(float *roll, float *pitch, float *yaw)
{
    *roll = atan2f(2.0f * (q2*q3 + q0*q1), q0*q0 - q1*q1 - q2*q2 + q3*q3) * 57.29578f;
    *pitch = asinf(-2.0f * (q1*q3 - q0*q2)) * 57.29578f;
    *yaw = atan2f(2.0f * (q1*q2 + q0*q3), q0*q0 + q1*q1 - q2*q2 - q3*q3) * 57.29578f;
}
