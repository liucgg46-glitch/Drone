#include "bmp280.h"
#include "bsp_i2c.h"
#include "bsp_timer.h"
#include "bsp_uart.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

/* ==================== 内部状态 ==================== */
static uint8_t g_bmp_addr = BMP280_ADDR_LOW;
static uint8_t g_bmp_chip_id = 0;
static uint8_t g_bmp_inited = 0;

static bmp280_calib_t g_bmp_calib;
static bmp280_data_t  g_bmp_data;

/* Bosch 补偿算法需要的中间量 */
static int32_t g_t_fine = 0;

/* 默认海平面气压，单位 Pa。可根据当地气压修改 */
static float g_sea_level_pa = 101325.0f;

/* ==================== 内部工具函数 ==================== */
static uint16_t BMP280_U16_LE(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static int16_t BMP280_S16_LE(const uint8_t *p)
{
    return (int16_t)BMP280_U16_LE(p);
}

static void BMP280_SimpleDelay(volatile uint32_t n)
{
    while (n--) {
        __NOP();
    }
}

static uint8_t BMP280_WriteReg(uint8_t reg, uint8_t value)
{
    return I2C1_WriteByte(g_bmp_addr, reg, value);
}

static uint8_t BMP280_ReadReg(uint8_t reg, uint8_t *value)
{
    return I2C1_ReadByte(g_bmp_addr, reg, value);
}

static uint8_t BMP280_ReadRegs(uint8_t reg, uint8_t *buf, uint16_t len)
{
    return I2C1_ReadBytes(g_bmp_addr, reg, buf, len);
}

static float BMP280_ComputeAltitude(float pressure_pa)
{
    double ratio;
    double altitude;

    if (pressure_pa <= 0.0f || g_sea_level_pa <= 0.0f) {
        return 0.0f;
    }

    ratio = (double)pressure_pa / (double)g_sea_level_pa;
    altitude = 44330.0 * (1.0 - pow(ratio, 0.19029495718));
    return (float)altitude;
}

/* 温度补偿：返回 0.01 摄氏度，例如 2534 = 25.34C */
static int32_t BMP280_CompensateTemperatureInt32(int32_t adc_T)
{
    int32_t var1;
    int32_t var2;
    int32_t T;

    var1 = ((((adc_T >> 3) - ((int32_t)g_bmp_calib.dig_T1 << 1))) *
            ((int32_t)g_bmp_calib.dig_T2)) >> 11;

    var2 = (((((adc_T >> 4) - ((int32_t)g_bmp_calib.dig_T1)) *
              ((adc_T >> 4) - ((int32_t)g_bmp_calib.dig_T1))) >> 12) *
            ((int32_t)g_bmp_calib.dig_T3)) >> 14;

    g_t_fine = var1 + var2;
    T = (g_t_fine * 5 + 128) >> 8;
    return T;
}

/* 气压补偿：返回 Q24.8 格式 Pa，即返回值 / 256 = Pa */
static uint32_t BMP280_CompensatePressureInt64(int32_t adc_P, uint8_t *err)
{
    int64_t var1;
    int64_t var2;
    int64_t p;

    if (err != 0) {
        *err = 0;
    }

    var1 = ((int64_t)g_t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)g_bmp_calib.dig_P6;
    var2 = var2 + ((var1 * (int64_t)g_bmp_calib.dig_P5) << 17);
    var2 = var2 + (((int64_t)g_bmp_calib.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)g_bmp_calib.dig_P3) >> 8) +
           ((var1 * (int64_t)g_bmp_calib.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1) *
            ((int64_t)g_bmp_calib.dig_P1)) >> 33;

    if (var1 == 0) {
        if (err != 0) {
            *err = 1;
        }
        return 0;
    }

    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)g_bmp_calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)g_bmp_calib.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)g_bmp_calib.dig_P7) << 4);

    return (uint32_t)p;
}

/* ==================== 对外接口 ==================== */
uint8_t BMP280_Detect(uint8_t *addr, uint8_t *chip_id)
{
    uint8_t id;

    /* 先试 0x76 */
    g_bmp_addr = BMP280_ADDR_LOW;
    if (BMP280_ReadReg(BMP280_REG_CHIP_ID, &id) == 0) {
        if (id == BMP280_CHIP_ID || id == BME280_CHIP_ID) {
            g_bmp_chip_id = id;
            if (addr != 0) *addr = g_bmp_addr;
            if (chip_id != 0) *chip_id = id;
            return BMP280_OK;
        }
    }

    /* 再试 0x77 */
    g_bmp_addr = BMP280_ADDR_HIGH;
    if (BMP280_ReadReg(BMP280_REG_CHIP_ID, &id) == 0) {
        if (id == BMP280_CHIP_ID || id == BME280_CHIP_ID) {
            g_bmp_chip_id = id;
            if (addr != 0) *addr = g_bmp_addr;
            if (chip_id != 0) *chip_id = id;
            return BMP280_OK;
        }
    }

    g_bmp_chip_id = 0;
    return BMP280_ERR_ID;
}

uint8_t BMP280_ReadCalibration(void)
{
    uint8_t buf[BMP280_REG_CALIB_LEN];

    if (BMP280_ReadRegs(BMP280_REG_CALIB_START, buf, BMP280_REG_CALIB_LEN) != 0) {
        return BMP280_ERR_I2C;
    }

    g_bmp_calib.dig_T1 = BMP280_U16_LE(&buf[0]);
    g_bmp_calib.dig_T2 = BMP280_S16_LE(&buf[2]);
    g_bmp_calib.dig_T3 = BMP280_S16_LE(&buf[4]);

    g_bmp_calib.dig_P1 = BMP280_U16_LE(&buf[6]);
    g_bmp_calib.dig_P2 = BMP280_S16_LE(&buf[8]);
    g_bmp_calib.dig_P3 = BMP280_S16_LE(&buf[10]);
    g_bmp_calib.dig_P4 = BMP280_S16_LE(&buf[12]);
    g_bmp_calib.dig_P5 = BMP280_S16_LE(&buf[14]);
    g_bmp_calib.dig_P6 = BMP280_S16_LE(&buf[16]);
    g_bmp_calib.dig_P7 = BMP280_S16_LE(&buf[18]);
    g_bmp_calib.dig_P8 = BMP280_S16_LE(&buf[20]);
    g_bmp_calib.dig_P9 = BMP280_S16_LE(&buf[22]);

    if (g_bmp_calib.dig_T1 == 0 || g_bmp_calib.dig_P1 == 0) {
        return BMP280_ERR_CALIB;
    }

    return BMP280_OK;
}

uint8_t BMP280_Config(bmp280_osrs_t osrs_t,
                      bmp280_osrs_t osrs_p,
                      bmp280_mode_t mode,
                      bmp280_filter_t filter,
                      bmp280_standby_t standby)
{
    uint8_t config;
    uint8_t ctrl_meas;

    config = (uint8_t)(((uint8_t)standby << 5) |
                       ((uint8_t)filter << 2));

    ctrl_meas = (uint8_t)(((uint8_t)osrs_t << 5) |
                          ((uint8_t)osrs_p << 2) |
                          ((uint8_t)mode));

    /* datasheet 建议 normal mode 下先写 config，再写 ctrl_meas */
    if (BMP280_WriteReg(BMP280_REG_CONFIG, config) != 0) {
        return BMP280_ERR_I2C;
    }

    if (BMP280_WriteReg(BMP280_REG_CTRL_MEAS, ctrl_meas) != 0) {
        return BMP280_ERR_I2C;
    }

    return BMP280_OK;
}

uint8_t BMP280_Init(void)
{
    uint8_t ret;

    memset(&g_bmp_calib, 0, sizeof(g_bmp_calib));
    memset(&g_bmp_data, 0, sizeof(g_bmp_data));
    g_bmp_inited = 0;
    g_t_fine = 0;

    ret = BMP280_Detect(&g_bmp_addr, &g_bmp_chip_id);
    if (ret != BMP280_OK) {
        return ret;
    }

    /* 软复位 */
    if (BMP280_WriteReg(BMP280_REG_RESET, BMP280_RESET_VALUE) != 0) {
        return BMP280_ERR_I2C;
    }
    BMP280_SimpleDelay(800000U);

    ret = BMP280_ReadCalibration();
    if (ret != BMP280_OK) {
        return ret;
    }

    /* 默认配置：温度 x1，气压 x4，normal mode，IIR x4，125ms standby */
    ret = BMP280_Config(BMP280_OSRS_X1,
                        BMP280_OSRS_X4,
                        BMP280_MODE_NORMAL,
                        BMP280_FILTER_X4,
                        BMP280_STANDBY_125_MS);
    if (ret != BMP280_OK) {
        return ret;
    }

    g_bmp_inited = 1;
    return BMP280_OK;
}

uint8_t BMP280_ReadRaw(bmp280_raw_t *raw)
{
    uint8_t buf[BMP280_REG_PRESS_LEN];
    int32_t adc_P;
    int32_t adc_T;

    if (raw == 0) {
        return BMP280_ERR_NULL;
    }

    if (!g_bmp_inited) {
        return BMP280_ERR_ID;
    }

    if (BMP280_ReadRegs(BMP280_REG_PRESS_MSB, buf, BMP280_REG_PRESS_LEN) != 0) {
        return BMP280_ERR_I2C;
    }

    adc_P = ((int32_t)buf[0] << 12) | ((int32_t)buf[1] << 4) | ((int32_t)buf[2] >> 4);
    adc_T = ((int32_t)buf[3] << 12) | ((int32_t)buf[4] << 4) | ((int32_t)buf[5] >> 4);

    raw->adc_P = adc_P;
    raw->adc_T = adc_T;

    return BMP280_OK;
}

uint8_t BMP280_ReadData(bmp280_data_t *out)
{
    bmp280_raw_t raw;
    int32_t temp_x100;
    uint32_t press_q24_8;
    uint8_t div_err = 0;
    float pressure_pa;

    if (!g_bmp_inited) {
        return BMP280_ERR_ID;
    }

    if (BMP280_ReadRaw(&raw) != BMP280_OK) {
        return BMP280_ERR_I2C;
    }

    temp_x100 = BMP280_CompensateTemperatureInt32(raw.adc_T);
    press_q24_8 = BMP280_CompensatePressureInt64(raw.adc_P, &div_err);
    if (div_err) {
        return BMP280_ERR_DIV_ZERO;
    }

    pressure_pa = ((float)press_q24_8) / 256.0f;

    g_bmp_data.temperature_c = ((float)temp_x100) / 100.0f;
    g_bmp_data.pressure_pa = pressure_pa;
    g_bmp_data.pressure_hpa = pressure_pa / 100.0f;
    g_bmp_data.altitude_m = BMP280_ComputeAltitude(pressure_pa);
    g_bmp_data.adc_T = raw.adc_T;
    g_bmp_data.adc_P = raw.adc_P;
    g_bmp_data.stamp_ms = GetTick();
    g_bmp_data.valid = 1;

    if (out != 0) {
        *out = g_bmp_data;
    }

    return BMP280_OK;
}

void BMP280_SetSeaLevelPressure(float pressure_pa)
{
    if (pressure_pa > 80000.0f && pressure_pa < 120000.0f) {
        g_sea_level_pa = pressure_pa;
    }
}

float BMP280_GetSeaLevelPressure(void)
{
    return g_sea_level_pa;
}

uint8_t BMP280_GetAddress(void)
{
    return g_bmp_addr;
}

uint8_t BMP280_GetChipId(void)
{
    return g_bmp_chip_id;
}

uint8_t BMP280_IsInitialized(void)
{
    return g_bmp_inited;
}

const bmp280_calib_t *BMP280_GetCalibration(void)
{
    return &g_bmp_calib;
}

const bmp280_data_t *BMP280_GetData(void)
{
    return &g_bmp_data;
}

void BMP280_DebugPrintDetect(void)
{
    uint8_t addr;
    uint8_t chip_id;
    char buf[96];
    uint8_t ret;

    ret = BMP280_Detect(&addr, &chip_id);
    if (ret == BMP280_OK) {
        if (chip_id == BMP280_CHIP_ID) {
            snprintf(buf, sizeof(buf),
                     "BMP280 found: addr=0x%02X chip_id=0x%02X\r\n",
                     addr, chip_id);
        } else {
            snprintf(buf, sizeof(buf),
                     "BME280 found: addr=0x%02X chip_id=0x%02X\r\n",
                     addr, chip_id);
        }
    } else {
        snprintf(buf, sizeof(buf), "BMP280 not found at 0x76 or 0x77\r\n");
    }

    UART1_SendData_NonBlocking((uint8_t*)buf, strlen(buf));
}

void BMP280_DebugPrintData(void)
{
    bmp280_data_t d;
    char buf[128];
    uint8_t ret;

    ret = BMP280_ReadData(&d);
    if (ret == BMP280_OK) {
        snprintf(buf, sizeof(buf),
                 "BMP T:%6.2fC P:%8.2fPa %7.2fhPa H:%7.2fm\r\n",
                 d.temperature_c,
                 d.pressure_pa,
                 d.pressure_hpa,
                 d.altitude_m);
    } else {
        snprintf(buf, sizeof(buf), "BMP280 read fail: ret=%u\r\n", ret);
    }

    UART1_SendData_NonBlocking((uint8_t*)buf, strlen(buf));
}
