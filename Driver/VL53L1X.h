#ifndef __VL53L1X_H
#define __VL53L1X_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==================== VL53L1X 基本信息 ====================
 * 注意：VL53L1X 使用 16-bit 寄存器地址，不适合直接调用当前 bsp_i2c.c
 * 里只有 8-bit reg_addr 的 I2C1_ReadBytes/I2C1_WriteBytes。
 */
#define VL53L1X_ADDR_DEFAULT                 0x29U   /* 7-bit I2C address */

#define VL53L1X_REG_IDENTIFICATION_MODEL_ID 0x010FU
#define VL53L1X_EXPECTED_MODEL_ID           0xEAU
#define VL53L1X_EXPECTED_MODULE_TYPE        0xCCU

/* ==================== 返回值 ==================== */
typedef enum {
    VL53L1X_OK = 0,
    VL53L1X_ERR_NULL = 1,
    VL53L1X_ERR_I2C = 2,
    VL53L1X_ERR_ID = 3,
    VL53L1X_ERR_BUSY = 4
} vl53l1x_status_t;

/* ==================== ID 信息 ==================== */
typedef struct {
    uint8_t address;
    uint8_t model_id;       /* 正常 VL53L1X 一般为 0xEA */
    uint8_t module_type;    /* 常见为 0xCC */
    uint8_t present;
} vl53l1x_id_t;

/* ==================== 底层 16-bit 寄存器访问接口 ==================== */
uint8_t VL53L1X_ReadMulti(uint16_t reg, uint8_t *buf, uint16_t len);
uint8_t VL53L1X_WriteMulti(uint16_t reg, const uint8_t *buf, uint16_t len);

uint8_t VL53L1X_ReadReg8(uint16_t reg, uint8_t *value);
uint8_t VL53L1X_ReadReg16(uint16_t reg, uint16_t *value);
uint8_t VL53L1X_ReadReg32(uint16_t reg, uint32_t *value);

uint8_t VL53L1X_WriteReg8(uint16_t reg, uint8_t value);
uint8_t VL53L1X_WriteReg16(uint16_t reg, uint16_t value);
uint8_t VL53L1X_WriteReg32(uint16_t reg, uint32_t value);

/* ==================== 连通性测试接口 ==================== */
uint8_t VL53L1X_Detect(vl53l1x_id_t *id);
uint8_t VL53L1X_Init(void);        /* 当前版本只做 detect，不做正式测距初始化 */

uint8_t VL53L1X_IsPresent(void);
uint8_t VL53L1X_GetAddress(void);
uint8_t VL53L1X_GetModelId(void);
uint8_t VL53L1X_GetModuleType(void);
const vl53l1x_id_t *VL53L1X_GetIdInfo(void);

void VL53L1X_DebugPrintDetect(void);

#ifdef __cplusplus
}
#endif

#endif /* __VL53L1X_H */
