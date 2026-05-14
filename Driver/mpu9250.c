#include "mpu9250.h"
#include "bsp_i2c.h"
#include "bsp_timer.h"
#include <string.h>

/*
 * 说明：
 * 1. 初始化阶段使用少量阻塞 I2C 写寄存器，简单可靠。
 * 2. 飞控运行阶段使用 MPU9250_ReadAccelGyroTemp_DMA() 非阻塞读取 14 字节。
 * 3. DMA callback 在中断上下文中执行，只置标志，不要在 callback 里做串口打印/姿态解算。
 */

#define MPU9250_FRAME_LEN       14U

static uint8_t mpu_rx_buf[MPU9250_FRAME_LEN];
static uint8_t mpu_id_buf = 0;

static volatile uint8_t mpu_busy = 0;
static volatile uint8_t mpu_data_ready = 0;

static mpu9250_raw_t mpu_raw;
static MPU9250_Callback  mpu_user_cb = 0;
static MPU9250_IdCallback mpu_id_user_cb = 0;

static int16_t MPU9250_ReadInt16BE(const uint8_t *p)
{
    return (int16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

static void MPU9250_SimpleDelay(volatile uint32_t n)
{
    while (n--) {
        __NOP();
    }
}

static void MPU9250_ParseFrame(const uint8_t *buf)
{
    if (buf == 0) {
        return;
    }

    mpu_raw.ax       = MPU9250_ReadInt16BE(&buf[0]);
    mpu_raw.ay       = MPU9250_ReadInt16BE(&buf[2]);
    mpu_raw.az       = MPU9250_ReadInt16BE(&buf[4]);
    mpu_raw.temp_raw = MPU9250_ReadInt16BE(&buf[6]);
    mpu_raw.gx       = MPU9250_ReadInt16BE(&buf[8]);
    mpu_raw.gy       = MPU9250_ReadInt16BE(&buf[10]);
    mpu_raw.gz       = MPU9250_ReadInt16BE(&buf[12]);

    /* MPU9250 温度公式：Temp_degC = temp_raw / 333.87 + 21 */
    mpu_raw.temp_c   = ((float)mpu_raw.temp_raw) / 333.87f + 21.0f;
    mpu_raw.stamp_ms = GetTick();
    mpu_raw.valid    = 1;
}

static uint8_t MPU9250_WriteReg(uint8_t reg, uint8_t value)
{
    return I2C1_WriteByte(MPU9250_ADDR, reg, value);
}

/* ==================== 初始化与阻塞读接口 ==================== */
uint8_t MPU9250_ReadWhoAmI(uint8_t *whoami)
{
    if (whoami == 0) {
        return 1;
    }

    return I2C1_ReadByte(MPU9250_ADDR, MPU9250_REG_WHO_AM_I, whoami);
}

uint8_t MPU9250_Init(mpu9250_gyro_fs_t gyro_fs, mpu9250_accel_fs_t accel_fs)
{
    uint8_t id = 0;

    memset(&mpu_raw, 0, sizeof(mpu_raw));
    mpu_busy = 0;
    mpu_data_ready = 0;

    /* 读取 WHO_AM_I，只要 I2C 能读通就继续。
     * 你当前实测 WHO_AM_I = 0x70，所以这里不强制只认 0x71。
     */
    if (MPU9250_ReadWhoAmI(&id) != 0) {
        return 1;
    }

    /* 复位芯片 */
    if (MPU9250_WriteReg(MPU9250_REG_PWR_MGMT_1, 0x80U) != 0) return 2;
    MPU9250_SimpleDelay(1000000U);

    /* 选择 PLL 时钟，唤醒所有轴 */
    if (MPU9250_WriteReg(MPU9250_REG_PWR_MGMT_1, 0x01U) != 0) return 3;
    if (MPU9250_WriteReg(MPU9250_REG_PWR_MGMT_2, 0x00U) != 0) return 4;
    MPU9250_SimpleDelay(200000U);

    /* DLPF 与采样率：陀螺仪内部 1kHz，SMPLRT_DIV=4 => 200Hz */
    if (MPU9250_WriteReg(MPU9250_REG_CONFIG, 0x03U) != 0) return 5;
    if (MPU9250_WriteReg(MPU9250_REG_SMPLRT_DIV, 0x04U) != 0) return 6;

    /* 量程 */
    if (MPU9250_WriteReg(MPU9250_REG_GYRO_CONFIG,  (uint8_t)gyro_fs) != 0) return 7;
    if (MPU9250_WriteReg(MPU9250_REG_ACCEL_CONFIG, (uint8_t)accel_fs) != 0) return 8;

    /* 加速度 DLPF */
    if (MPU9250_WriteReg(MPU9250_REG_ACCEL_CONFIG2, 0x03U) != 0) return 9;

    /* 关闭 I2C 主机模式，打开 bypass，后续磁力计 AK8963 可直接访问 0x0C */
    if (MPU9250_WriteReg(MPU9250_REG_USER_CTRL, 0x00U) != 0) return 10;
    if (MPU9250_WriteReg(MPU9250_REG_INT_PIN_CFG, 0x02U) != 0) return 11;

    return 0;
}

uint8_t MPU9250_ReadAccelGyroTemp(int16_t accel[3], int16_t gyro[3], int16_t *temp_raw)
{
    uint8_t buf[MPU9250_FRAME_LEN];

    if (I2C1_ReadBytes(MPU9250_ADDR, MPU9250_REG_ACCEL_XOUT_H, buf, MPU9250_FRAME_LEN) != 0) {
        return 1;
    }

    MPU9250_ParseFrame(buf);

    if (accel != 0) {
        accel[0] = mpu_raw.ax;
        accel[1] = mpu_raw.ay;
        accel[2] = mpu_raw.az;
    }

    if (gyro != 0) {
        gyro[0] = mpu_raw.gx;
        gyro[1] = mpu_raw.gy;
        gyro[2] = mpu_raw.gz;
    }

    if (temp_raw != 0) {
        *temp_raw = mpu_raw.temp_raw;
    }

    mpu_data_ready = 1;
    return 0;
}

uint8_t MPU9250_ReadAccelGyro(int16_t accel[3], int16_t gyro[3])
{
    return MPU9250_ReadAccelGyroTemp(accel, gyro, 0);
}

uint8_t MPU9250_ReadTemp(int16_t *temp_raw)
{
    uint8_t buf[2];

    if (temp_raw == 0) {
        return 1;
    }

    if (I2C1_ReadBytes(MPU9250_ADDR, MPU9250_REG_TEMP_OUT_H, buf, 2) != 0) {
        return 2;
    }

    *temp_raw = MPU9250_ReadInt16BE(buf);
    return 0;
}

/* ==================== DMA 非阻塞读接口 ==================== */
static void MPU9250_I2C_DataCallback(int result)
{
    MPU9250_Callback cb;

    cb = mpu_user_cb;
    mpu_user_cb = 0;
    mpu_busy = 0;

    if (result == 0) {
        MPU9250_ParseFrame(mpu_rx_buf);
        mpu_data_ready = 1;
    }

    if (cb != 0) {
        cb(result, &mpu_raw);
    }
}

static void MPU9250_I2C_IdCallback(int result)
{
    MPU9250_IdCallback cb;

    cb = mpu_id_user_cb;
    mpu_id_user_cb = 0;
    mpu_busy = 0;

    if (cb != 0) {
        cb(result, mpu_id_buf);
    }
}

uint8_t MPU9250_ReadWhoAmI_DMA(MPU9250_IdCallback callback)
{
    uint8_t ret;

    if (mpu_busy) {
        return 1;
    }

    mpu_busy = 1;
    mpu_id_user_cb = callback;
    mpu_id_buf = 0;

    ret = I2C1_ReadBytes_DMA_Async(MPU9250_ADDR,
                                   MPU9250_REG_WHO_AM_I,
                                   &mpu_id_buf,
                                   1,
                                   MPU9250_I2C_IdCallback);
    if (ret != 0) {
        mpu_busy = 0;
        mpu_id_user_cb = 0;
    }

    return ret;
}

uint8_t MPU9250_ReadAccelGyroTemp_DMA(MPU9250_Callback callback)
{
    uint8_t ret;

    if (mpu_busy) {
        return 1;
    }

    mpu_busy = 1;
    mpu_user_cb = callback;

    ret = I2C1_ReadBytes_DMA_Async(MPU9250_ADDR,
                                   MPU9250_REG_ACCEL_XOUT_H,
                                   mpu_rx_buf,
                                   MPU9250_FRAME_LEN,
                                   MPU9250_I2C_DataCallback);
    if (ret != 0) {
        mpu_busy = 0;
        mpu_user_cb = 0;
    }

    return ret;
}

uint8_t MPU9250_ReadAccelGyro_DMA(MPU9250_Callback callback)
{
    return MPU9250_ReadAccelGyroTemp_DMA(callback);
}

uint8_t MPU9250_IsBusy(void)
{
    return mpu_busy;
}

uint8_t MPU9250_DataAvailable(void)
{
    return mpu_data_ready;
}

void MPU9250_ClearDataAvailable(void)
{
    mpu_data_ready = 0;
}

const mpu9250_raw_t *MPU9250_GetRawData(void)
{
    return &mpu_raw;
}

