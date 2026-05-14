#include "quaternion.h"
#include <math.h>

/* 圆周率，用于弧度和角度之间的转换 */
#define Q_PI 3.14159265358979323846f

/* 弧度转角度系数：deg = rad * 180 / pi */
#define RAD_TO_DEG (180.0f / Q_PI)

/*
 * 函数：quat_set_identity
 * 作用：设置单位四元数
 * 含义：单位四元数表示“没有旋转”，通常作为初始姿态
 */
void quat_set_identity(quat_t *q)
{
    /* 空指针保护，防止传入 NULL 导致程序崩溃 */
    if (!q) return;

    /* 单位四元数：[w, x, y, z] = [1, 0, 0, 0] */
    q->w = 1.0f;
    q->x = 0.0f;
    q->y = 0.0f;
    q->z = 0.0f;
}

/*
 * 函数：quat_normalize
 * 作用：四元数归一化
 * 原因：姿态四元数必须满足 w^2 + x^2 + y^2 + z^2 = 1
 *       由于积分和浮点误差，四元数长度可能偏离 1，因此需要修正
 */
void quat_normalize(quat_t *q)
{
    float n;

    /* 空指针保护 */
    if (!q) return;

    /* 计算四元数模长 n = sqrt(w^2 + x^2 + y^2 + z^2) */
    n = sqrtf(q->w*q->w + q->x*q->x + q->y*q->y + q->z*q->z);

    /*
     * 如果模长太小，说明四元数已经无效。
     * 为避免除以接近 0 的数，直接恢复为单位四元数。
     */
    if (n < 1.0e-9f) {
        quat_set_identity(q);
        return;
    }

    /* 每个分量都除以模长，使四元数长度重新变为 1 */
    q->w /= n;
    q->x /= n;
    q->y /= n;
    q->z /= n;
}

/*
 * 函数：quat_multiply
 * 作用：四元数乘法
 * 含义：组合两个旋转
 * 例子：当前姿态 a，再叠加一个旋转 b，得到新姿态 r
 */
quat_t quat_multiply(quat_t a, quat_t b)
{
    quat_t r;

    /*
     * 四元数乘法公式。
     * 注意：四元数乘法不满足交换律，a*b 和 b*a 通常不一样。
     */
    r.w = a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z;
    r.x = a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y;
    r.y = a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x;
    r.z = a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w;

    return r;
}

/*
 * 函数：quat_to_euler_deg
 * 作用：将四元数转换成欧拉角
 * 输出：roll、pitch、yaw，单位为度 deg
 * 用途：给串口调试、姿态控制 PID 使用
 */
euler_t quat_to_euler_deg(quat_t q)
{
    euler_t e;

    /* 计算 roll 时用到的中间量 */
    float sinr_cosp, cosr_cosp;

    /* 计算 pitch 时用到的 sin(pitch) */
    float sinp;

    /* 计算 yaw 时用到的中间量 */
    float siny_cosp, cosy_cosp;

    /*
     * 先归一化四元数，保证转换结果稳定。
     * 这里 q 是传值进来的副本，所以不会改变外部原始四元数。
     */
    quat_normalize(&q);

    /*
     * 计算 Roll 横滚角
     * Roll 表示无人机左右倾斜角
     */
    sinr_cosp = 2.0f * (q.w*q.x + q.y*q.z);
    cosr_cosp = 1.0f - 2.0f * (q.x*q.x + q.y*q.y);
    e.roll = atan2f(sinr_cosp, cosr_cosp) * RAD_TO_DEG;

    /*
     * 计算 Pitch 俯仰角
     * Pitch 表示无人机机头抬起或低头的角度
     */
    sinp = 2.0f * (q.w*q.y - q.z*q.x);

    /*
     * 由于浮点误差，sinp 可能略微超过 [-1, 1]。
     * asinf() 的输入必须在 [-1, 1]，所以这里做限幅。
     */
    if (sinp > 1.0f) sinp = 1.0f;
    if (sinp < -1.0f) sinp = -1.0f;

    e.pitch = asinf(sinp) * RAD_TO_DEG;

    /*
     * 计算 Yaw 偏航角
     * Yaw 表示无人机机头在水平面内的朝向
     */
    siny_cosp = 2.0f * (q.w*q.z + q.x*q.y);
    cosy_cosp = 1.0f - 2.0f * (q.y*q.y + q.z*q.z);
    e.yaw = atan2f(siny_cosp, cosy_cosp) * RAD_TO_DEG;

    return e;
}