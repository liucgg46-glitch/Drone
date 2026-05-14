#ifndef __BMP280_H
#define __BMP280_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==================== BMP280 I2C 地址与 ID ==================== */
#define BMP280_ADDR_LOW             0x76U
#define BMP280_ADDR_HIGH            0x77U
#define BMP280_CHIP_ID              0x58U
#define BME280_CHIP_ID              0x60U   /* BME280 的温压部分与 BMP280 基本兼容 */

/* ==================== BMP280 寄存器 ==================== */
#define BMP280_REG_CALIB_START      0x88U
#define BMP280_REG_CALIB_LEN        24U
#define BMP280_REG_CHIP_ID          0xD0U
#define BMP280_REG_RESET            0xE0U
#define BMP280_REG_STATUS           0xF3U
#define BMP280_REG_CTRL_MEAS        0xF4U
#define BMP280_REG_CONFIG           0xF5U
#define BMP280_REG_PRESS_MSB        0xF7U
#define BMP280_REG_PRESS_LEN        6U

#define BMP280_RESET_VALUE          0xB6U

/* ==================== 返回值 ==================== */
typedef enum {
    BMP280_OK = 0,
    BMP280_ERR_NULL = 1,
    BMP280_ERR_I2C = 2,
    BMP280_ERR_ID = 3,
    BMP280_ERR_CALIB = 4,
    BMP280_ERR_DIV_ZERO = 5
} bmp280_status_t;

/* ==================== 工作模式 ==================== */
typedef enum {
    BMP280_MODE_SLEEP  = 0x00U,
    BMP280_MODE_FORCED = 0x01U,
    BMP280_MODE_NORMAL = 0x03U
} bmp280_mode_t;

typedef enum {
    BMP280_OSRS_SKIP = 0x00U,
    BMP280_OSRS_X1   = 0x01U,
    BMP280_OSRS_X2   = 0x02U,
    BMP280_OSRS_X4   = 0x03U,
    BMP280_OSRS_X8   = 0x04U,
    BMP280_OSRS_X16  = 0x05U
} bmp280_osrs_t;

typedef enum {
    BMP280_FILTER_OFF = 0x00U,
    BMP280_FILTER_X2  = 0x01U,
    BMP280_FILTER_X4  = 0x02U,
    BMP280_FILTER_X8  = 0x03U,
    BMP280_FILTER_X16 = 0x04U
} bmp280_filter_t;

typedef enum {
    BMP280_STANDBY_0_5_MS  = 0x00U,
    BMP280_STANDBY_62_5_MS = 0x01U,
    BMP280_STANDBY_125_MS  = 0x02U,
    BMP280_STANDBY_250_MS  = 0x03U,
    BMP280_STANDBY_500_MS  = 0x04U,
    BMP280_STANDBY_1000_MS = 0x05U,
    BMP280_STANDBY_2000_MS = 0x06U,
    BMP280_STANDBY_4000_MS = 0x07U
} bmp280_standby_t;

/* ==================== 校准参数 ==================== */
typedef struct {
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;

    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;
} bmp280_calib_t;

/* ==================== 原始数据 ==================== */
typedef struct {
    int32_t adc_T;
    int32_t adc_P;
} bmp280_raw_t;

/* ==================== 换算后的物理量 ==================== */
typedef struct {
    float temperature_c;     /* 摄氏度 */
    float pressure_pa;       /* Pa */
    float pressure_hpa;      /* hPa */
    float altitude_m;        /* 按海平面气压估算的相对高度 */

    int32_t adc_T;
    int32_t adc_P;
    uint32_t stamp_ms;
    uint8_t valid;
} bmp280_data_t;

/* ==================== 对外接口 ==================== */
uint8_t BMP280_Detect(uint8_t *addr, uint8_t *chip_id);
uint8_t BMP280_Init(void);
uint8_t BMP280_Config(bmp280_osrs_t osrs_t,
                      bmp280_osrs_t osrs_p,
                      bmp280_mode_t mode,
                      bmp280_filter_t filter,
                      bmp280_standby_t standby);

uint8_t BMP280_ReadCalibration(void);
uint8_t BMP280_ReadRaw(bmp280_raw_t *raw);
uint8_t BMP280_ReadData(bmp280_data_t *out);

void BMP280_SetSeaLevelPressure(float pressure_pa);
float BMP280_GetSeaLevelPressure(void);

uint8_t BMP280_GetAddress(void);
uint8_t BMP280_GetChipId(void);
uint8_t BMP280_IsInitialized(void);

const bmp280_calib_t *BMP280_GetCalibration(void);
const bmp280_data_t  *BMP280_GetData(void);

/* 调试用：兼容你前面 ID 测试代码 */
void BMP280_DebugPrintDetect(void);
void BMP280_DebugPrintData(void);

#ifdef __cplusplus
}
#endif

#endif /* __BMP280_H */
