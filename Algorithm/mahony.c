#include "mahony.h"
#include <math.h>

/*
 * 函数：normalize3
 * -------------------------------------------------
 * 作用：
 *   将三维向量归一化为单位向量。
 *
 * 用途：
 *   在 Mahony 算法中，加速度计主要用于提供“重力方向”。
 *   所以我们不关心加速度大小，只关心方向。
 *
 * 输入：
 *   x, y, z 三个分量的地址
 *
 * 输出：
 *   归一化后的 x, y, z
 *
 * 返回：
 *   1 -> 归一化成功
 *   0 -> 向量长度太小，无法归一化
 */
static int normalize3(float *x, float *y, float *z)
{
    float n = sqrtf((*x)*(*x) + (*y)*(*y) + (*z)*(*z));

    /*
     * 如果向量长度接近 0，不能除以 n。
     * 例如 ax=ay=az=0 时，说明加速度计数据无效。
     */
    if (n < 1.0e-9f) return 0;

    /*
     * 每个分量都除以模长，使向量长度变为 1。
     */
    *x /= n;
    *y /= n;
    *z /= n;

    return 1;
}

/*
 * 函数：Mahony_Init
 * -------------------------------------------------
 * 作用：
 *   初始化 Mahony 姿态解算器。
 *
 * 参数：
 *   m  -> Mahony 对象指针
 *   kp -> 比例修正增益
 *   ki -> 积分修正增益
 */
void Mahony_Init(mahony_t *m, float kp, float ki)
{
    /*
     * 空指针保护。
     */
    if (!m) return;

    /*
     * 保存 Mahony 的两个核心参数。
     * kp：当前误差的快速修正力度
     * ki：长期误差的积分修正力度
     */
    m->kp = kp;
    m->ki = ki;

    /*
     * 清空三轴积分误差。
     */
    m->ex_int = 0.0f;
    m->ey_int = 0.0f;
    m->ez_int = 0.0f;

    /*
     * 设置初始四元数为单位四元数。
     * 表示初始状态没有旋转，即 roll/pitch/yaw 接近 0。
     */
    quat_set_identity(&m->q);

    /*
     * 将初始四元数转换为欧拉角。
     */
    m->euler = quat_to_euler_deg(m->q);
}

/*
 * 函数：Mahony_UpdateIMU
 * -------------------------------------------------
 * 作用：
 *   使用 6轴 IMU 数据更新姿态。
 *
 * 使用数据：
 *   陀螺仪 gx/gy/gz
 *   加速度计 ax/ay/az
 *
 * 不使用：
 *   磁力计
 *
 * 单位要求：
 *   gx_rad/gy_rad/gz_rad -> rad/s
 *   ax_g/ay_g/az_g       -> g
 *   dt                   -> 秒
 */
void Mahony_UpdateIMU(mahony_t *m,
                      float gx_rad, float gy_rad, float gz_rad,
                      float ax_g, float ay_g, float az_g,
                      float dt)
{
    /*
     * q0 q1 q2 q3 是当前姿态四元数的四个分量。
     * q0 = w
     * q1 = x
     * q2 = y
     * q3 = z
     */
    float q0, q1, q2, q3;

    /*
     * vx vy vz：
     * 由当前四元数估计出来的重力方向。
     */
    float vx, vy, vz;

    /*
     * ex ey ez：
     * 实际重力方向和估计重力方向之间的误差。
     */
    float ex, ey, ez;

    /*
     * 四元数积分中使用 0.5 * dt。
     */
    float half_dt;

    /*
     * 参数保护：
     * m 为空，或者 dt 非法，则不更新。
     */
    if (!m || dt <= 0.0f) return;

    /*
     * 取出当前四元数，方便后面计算。
     */
    q0 = m->q.w;
    q1 = m->q.x;
    q2 = m->q.y;
    q3 = m->q.z;

    /*
     * 归一化加速度计数据。
     * 成功后，ax_g/ay_g/az_g 只表示方向，不表示大小。
     */
    if (normalize3(&ax_g, &ay_g, &az_g)) {

        /*
         * 根据当前四元数估计重力方向。
         *
         * 可以理解为：
         * “如果当前姿态估计是 q，那么重力在机体坐标系下应该是 vx/vy/vz。”
         */
        vx = 2.0f * (q1*q3 - q0*q2);
        vy = 2.0f * (q0*q1 + q2*q3);
        vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

        /*
         * 计算误差：
         * measured gravity × estimated gravity
         *
         * 即：
         * 实际加速度计测到的重力方向
         * 和
         * 当前四元数估计出来的重力方向
         * 之间的差异。
         *
         * 这个误差会用于修正陀螺仪。
         */
        ex = ay_g*vz - az_g*vy;
        ey = az_g*vx - ax_g*vz;
        ez = ax_g*vy - ay_g*vx;

        /*
         * 积分修正：
         * 如果 ki > 0，说明启用积分项。
         *
         * 积分项主要用于长期修正陀螺仪零漂。
         */
        if (m->ki > 0.0f) {
            m->ex_int += m->ki * ex * dt;
            m->ey_int += m->ki * ey * dt;
            m->ez_int += m->ki * ez * dt;

            /*
             * 把积分误差加到陀螺仪角速度上，
             * 实现对陀螺仪零漂的长期补偿。
             */
            gx_rad += m->ex_int;
            gy_rad += m->ey_int;
            gz_rad += m->ez_int;
        } else {

            /*
             * 如果不启用积分修正，就清空积分项，
             * 防止旧的积分误差残留。
             */
            m->ex_int = 0.0f;
            m->ey_int = 0.0f;
            m->ez_int = 0.0f;
        }

        /*
         * 比例修正：
         * 当前误差越大，立即修正越强。
         *
         * 这一步是 Mahony 快速纠正姿态漂移的核心。
         */
        gx_rad += m->kp * ex;
        gy_rad += m->kp * ey;
        gz_rad += m->kp * ez;
    }

    /*
     * 四元数积分更新。
     *
     * 本质：
     * 当前姿态 + 角速度 * 时间 = 新姿态
     *
     * 四元数微分方程中有 0.5 系数，
     * 所以这里使用 half_dt = 0.5 * dt。
     */
    half_dt = 0.5f * dt;

    m->q.w += (-q1*gx_rad - q2*gy_rad - q3*gz_rad) * half_dt;
    m->q.x += ( q0*gx_rad + q2*gz_rad - q3*gy_rad) * half_dt;
    m->q.y += ( q0*gy_rad - q1*gz_rad + q3*gx_rad) * half_dt;
    m->q.z += ( q0*gz_rad + q1*gy_rad - q2*gx_rad) * half_dt;

    /*
     * 积分后四元数长度可能偏离 1，
     * 所以每次更新后都需要归一化。
     */
    quat_normalize(&m->q);

    /*
     * 将四元数转换为欧拉角。
     * 后续 PID 控制通常使用 roll/pitch/yaw。
     */
    m->euler = quat_to_euler_deg(m->q);
}

/*
 * 函数：Mahony_UpdateMARG
 * -------------------------------------------------
 * 作用：
 *   理论上用于 9轴融合：
 *   陀螺仪 + 加速度计 + 磁力计
 *
 * 当前版本：
 *   暂时不使用磁力计，只调用 Mahony_UpdateIMU。
 *
 * 原因：
 *   无人机上的磁力计很容易受电机、电调、电源线干扰。
 *   如果没有完成硬铁/软铁校准，磁力计可能会破坏 yaw。
 */
void Mahony_UpdateMARG(mahony_t *m,
                       float gx_rad, float gy_rad, float gz_rad,
                       float ax_g, float ay_g, float az_g,
                       float mx, float my, float mz,
                       float dt)
{
    /*
     * 这三个参数当前版本故意不用。
     * 写成 (void) 是为了避免编译器产生“未使用参数”警告。
     */
    (void)mx;
    (void)my;
    (void)mz;

    /*
     * 当前版本仍然使用 6轴 IMU 更新姿态。
     */
    Mahony_UpdateIMU(m, gx_rad, gy_rad, gz_rad, ax_g, ay_g, az_g, dt);
}