#ifndef __MPU9250_H
#define __MPU9250_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==================== I2C 地址 ==================== */
#define MPU9250_ADDR            0x68U
#define AK8963_ADDR             0x0CU

/* ==================== MPU9250 寄存器 ==================== */
#define MPU9250_REG_SMPLRT_DIV      0x19U
#define MPU9250_REG_CONFIG          0x1AU
#define MPU9250_REG_GYRO_CONFIG     0x1BU
#define MPU9250_REG_ACCEL_CONFIG    0x1CU
#define MPU9250_REG_ACCEL_CONFIG2   0x1DU
#define MPU9250_REG_INT_PIN_CFG     0x37U
#define MPU9250_REG_ACCEL_XOUT_H    0x3BU
#define MPU9250_REG_TEMP_OUT_H      0x41U
#define MPU9250_REG_GYRO_XOUT_H     0x43U
#define MPU9250_REG_USER_CTRL       0x6AU
#define MPU9250_REG_PWR_MGMT_1      0x6BU
#define MPU9250_REG_PWR_MGMT_2      0x6CU
#define MPU9250_REG_WHO_AM_I        0x75U

/* ==================== 量程配置值，直接写入寄存器 ==================== */
typedef enum {
    GYRO_FS_250  = 0x00U,
    GYRO_FS_500  = 0x08U,
    GYRO_FS_1000 = 0x10U,
    GYRO_FS_2000 = 0x18U
} mpu9250_gyro_fs_t;

typedef enum {
    ACCEL_FS_2  = 0x00U,
    ACCEL_FS_4  = 0x08U,
    ACCEL_FS_8  = 0x10U,
    ACCEL_FS_16 = 0x18U
} mpu9250_accel_fs_t;

/* ==================== 原始数据结构 ==================== */
typedef struct {
    int16_t ax;
    int16_t ay;
    int16_t az;

    int16_t temp_raw;
    float   temp_c;

    int16_t gx;
    int16_t gy;
    int16_t gz;

    uint32_t stamp_ms;
    uint8_t  valid;
} mpu9250_raw_t;

/* result: 0=成功，负数=失败。注意：该回调在 I2C/DMA 中断上下文中执行，不要在里面 printf。 */
typedef void (*MPU9250_Callback)(int result, const mpu9250_raw_t *data);

typedef void (*MPU9250_IdCallback)(int result, uint8_t whoami);

/* ==================== 初始化与阻塞读接口 ==================== */
uint8_t MPU9250_Init(mpu9250_gyro_fs_t gyro_fs, mpu9250_accel_fs_t accel_fs);
uint8_t MPU9250_ReadWhoAmI(uint8_t *whoami);
uint8_t MPU9250_ReadAccelGyroTemp(int16_t accel[3], int16_t gyro[3], int16_t *temp_raw);
uint8_t MPU9250_ReadAccelGyro(int16_t accel[3], int16_t gyro[3]);
uint8_t MPU9250_ReadTemp(int16_t *temp_raw);

/* ==================== DMA 非阻塞读接口 ==================== */
uint8_t MPU9250_ReadWhoAmI_DMA(MPU9250_IdCallback callback);
uint8_t MPU9250_ReadAccelGyroTemp_DMA(MPU9250_Callback callback);
uint8_t MPU9250_ReadAccelGyro_DMA(MPU9250_Callback callback);  /* 等价于读 14 字节 */

uint8_t MPU9250_IsBusy(void);
uint8_t MPU9250_DataAvailable(void);
void MPU9250_ClearDataAvailable(void);
const mpu9250_raw_t *MPU9250_GetRawData(void);

#ifdef __cplusplus
}
#endif

#endif /* __MPU9250_H */
