
#ifndef __MAHONY_H
#define __MAHONY_H

/*
 * 引入四元数模块。
 * Mahony 姿态解算内部会使用：
 * 1. quat_t   -> 四元数姿态
 * 2. euler_t -> 欧拉角（Roll/Pitch/Yaw）
 */
#include "quaternion.h"


#ifdef __cplusplus
extern "C" {
#endif

/*
 * mahony_t
 * -----------------------------
 * 一个完整的 Mahony 姿态解算器对象。
 *
 * 里面包含：
 * 1. Mahony算法参数
 * 2. 误差积分项
 * 3. 当前四元数姿态
 * 4. 当前欧拉角姿态
 */
typedef struct {

    /*
     * 比例增益（Proportional Gain）
     * -----------------------------
     * 用于控制“加速度计修正陀螺仪”的力度。
     *
     * kp 越大：
     *   修正越快
     *   但更容易受震动影响
     *
     * kp 越小：
     *   更平滑
     *   但漂移修正慢
     */
    float kp;

    /*
     * 积分增益（Integral Gain）
     * -----------------------------
     * 用于长期修正陀螺仪零漂。
     *
     * 陀螺仪即使静止，也可能输出一点点角速度。
     * ki 会慢慢累计误差并补偿这种漂移。
     */
    float ki;

    /*
     * X轴误差积分
     * -----------------------------
     * 用于长期修正 X轴陀螺仪漂移。
     */
    float ex_int;

    /*
     * Y轴误差积分
     * -----------------------------
     * 用于长期修正 Y轴陀螺仪漂移。
     */
    float ey_int;

    /*
     * Z轴误差积分
     * -----------------------------
     * 用于长期修正 Z轴陀螺仪漂移。
     */
    float ez_int;

    /*
     * 当前姿态四元数
     * -----------------------------
     * 这是 Mahony 内部真正维护的姿态。
     *
     * 飞控内部一般不用 Roll/Pitch/Yaw 直接运算，
     * 而是使用四元数 q。
     *
     * 四元数不会发生欧拉角万向节死锁。
     */
    quat_t q;

    /*
     * 当前欧拉角
     * -----------------------------
     * 由四元数转换得到：
     *
     * roll  -> 横滚角
     * pitch -> 俯仰角
     * yaw   -> 偏航角
     *
     * 一般用于：
     * 1. 串口打印
     * 2. PID控制
     * 3. 上位机显示
     */
    euler_t euler;

} mahony_t;

/*
 * 函数：Mahony_Init
 * -------------------------------------------------
 * 作用：
 *   初始化一个 Mahony 姿态解算器。
 *
 * 参数：
 *   m  -> Mahony对象
 *   kp -> 比例修正增益
 *   ki -> 积分修正增益
 *
 * 典型调用：
 *
 *   mahony_t ahrs;
 *   Mahony_Init(&ahrs, 2.0f, 0.0f);
 */
void Mahony_Init(mahony_t *m, float kp, float ki);

/*
 * 函数：Mahony_UpdateIMU
 * -------------------------------------------------
 * 作用：
 *   使用 6轴 IMU 数据更新姿态。
 *
 * 使用传感器：
 *   1. 陀螺仪
 *   2. 加速度计
 *
 * 不使用磁力计。
 *
 * 输入参数：
 *
 * gx_rad / gy_rad / gz_rad
 * -----------------------------
 * 三轴陀螺仪角速度
 * 单位：rad/s（弧度每秒）
 *
 * 注意：
 * 如果 MPU9250 输出的是 deg/s（度每秒），
 * 必须先转换：
 *
 * rad/s = deg/s * PI / 180
 *
 * ax_g / ay_g / az_g
 * -----------------------------
 * 三轴加速度
 * 单位：g
 *
 * 例如静止水平放置时：
 * ax = 0
 * ay = 0
 * az = 1
 *
 * dt
 * -----------------------------
 * 采样周期
 * 单位：秒
 *
 * 例如：
 * 2ms = 0.002f
 *
 * 输出结果：
 * -----------------------------
 * 函数内部会更新：
 *
 * m->q
 * m->euler.roll
 * m->euler.pitch
 * m->euler.yaw
 *
 * 特点：
 * -----------------------------
 * Roll/Pitch 很稳定
 * Yaw 会慢慢漂移（因为没有磁力计）
 */
void Mahony_UpdateIMU(mahony_t *m,
                      float gx_rad, float gy_rad, float gz_rad,
                      float ax_g, float ay_g, float az_g,
                      float dt);

/*
 * 函数：Mahony_UpdateMARG
 * -------------------------------------------------
 * 作用：
 *   使用 9轴数据更新姿态。
 *
 * MARG：
 * Magnetic + Angular Rate + Gravity
 *
 * 即：
 *   磁力计
 * + 陀螺仪
 * + 加速度计
 *
 * 比 UpdateIMU 多了磁力计数据：
 *
 * mx / my / mz
 *
 * 这样可以利用地磁方向修正 Yaw 漂移。
 *
 * 参数：
 * -----------------------------
 *
 * gx_rad / gy_rad / gz_rad
 *   陀螺仪角速度（rad/s）
 *
 * ax_g / ay_g / az_g
 *   加速度（g）
 *
 * mx / my / mz
 *   三轴磁力计数据
 *
 * dt
 *   采样周期（秒）
 *
 * 特点：
 * -----------------------------
 * Roll/Pitch/Yaw 都能长期稳定
 *
 * 但无人机上磁力计容易受到：
 *   电机
 *   电调
 *   电源线
 * 的磁场干扰。
 *
 * 所以飞控开发初期，
 * 一般先用 Mahony_UpdateIMU()。
 */
void Mahony_UpdateMARG(mahony_t *m,
                       float gx_rad, float gy_rad, float gz_rad,
                       float ax_g, float ay_g, float az_g,
                       float mx, float my, float mz,
                       float dt);

/*
 * C++ 兼容结束
 */
#ifdef __cplusplus
}
#endif

/*
 * 头文件保护结束
 */
#endif