#include "vl53l1x.h"
#include "bsp_i2c.h"
#include "bsp_uart.h"
#include "stm32f4xx.h"
#include "stm32f4xx_i2c.h"
#include <stdio.h>
#include <string.h>

#define VL53L1X_I2C_TIMEOUT     3000UL

static vl53l1x_id_t g_vl53_id = { VL53L1X_ADDR_DEFAULT, 0, 0, 0 };

/* ==================== 内部工具函数 ==================== */
static uint8_t VL53L1X_WaitEvent(uint32_t event)
{
    uint32_t timeout = VL53L1X_I2C_TIMEOUT;

    while (!I2C_CheckEvent(I2C1, event)) {
        if (--timeout == 0) {
            return 0;
        }
    }

    return 1;
}

static uint8_t VL53L1X_WaitBusFree(void)
{
    uint32_t timeout = VL53L1X_I2C_TIMEOUT;

    while (I2C1->SR2 & I2C_SR2_BUSY) {
        if (--timeout == 0) {
            return 0;
        }
    }

    return 1;
}

static void VL53L1X_StopAndClear(void)
{
    I2C_GenerateSTOP(I2C1, ENABLE);
    I2C_AcknowledgeConfig(I2C1, ENABLE);
    I2C_ClearITPendingBit(I2C1, I2C_IT_BERR | I2C_IT_ARLO |
                                I2C_IT_AF   | I2C_IT_OVR);
}

static uint16_t VL53L1X_MakeU16BE(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

static uint32_t VL53L1X_MakeU32BE(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) |
           ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8)  |
           ((uint32_t)p[3]);
}

/* ==================== 16-bit 寄存器 I2C 读写 ==================== */
uint8_t VL53L1X_WriteMulti(uint16_t reg, const uint8_t *buf, uint16_t len)
{
    uint16_t i;

    if (buf == 0 || len == 0) {
        return VL53L1X_ERR_NULL;
    }

    /* 避免和 MPU6500 的 I2C DMA 状态机抢总线 */
    if (I2C1_IsBusy()) {
        return VL53L1X_ERR_BUSY;
    }

    if (!VL53L1X_WaitBusFree()) {
        return VL53L1X_ERR_BUSY;
    }

    I2C_GenerateSTART(I2C1, ENABLE);
    if (!VL53L1X_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT)) goto error;

    I2C_Send7bitAddress(I2C1, VL53L1X_ADDR_DEFAULT << 1, I2C_Direction_Transmitter);
    if (!VL53L1X_WaitEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) goto error;

    I2C_SendData(I2C1, (uint8_t)(reg >> 8));
    if (!VL53L1X_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) goto error;

    I2C_SendData(I2C1, (uint8_t)(reg & 0xFFU));
    if (!VL53L1X_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) goto error;

    for (i = 0; i < len; i++) {
        I2C_SendData(I2C1, buf[i]);
        if (!VL53L1X_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) goto error;
    }

    I2C_GenerateSTOP(I2C1, ENABLE);
    return VL53L1X_OK;

error:
    VL53L1X_StopAndClear();
    return VL53L1X_ERR_I2C;
}

uint8_t VL53L1X_ReadMulti(uint16_t reg, uint8_t *buf, uint16_t len)
{
    uint16_t i;

    if (buf == 0 || len == 0) {
        return VL53L1X_ERR_NULL;
    }

    /* 避免和 MPU6500 的 I2C DMA 状态机抢总线 */
    if (I2C1_IsBusy()) {
        return VL53L1X_ERR_BUSY;
    }

    if (!VL53L1X_WaitBusFree()) {
        return VL53L1X_ERR_BUSY;
    }

    I2C_AcknowledgeConfig(I2C1, ENABLE);

    /* 先写 16-bit 寄存器地址 */
    I2C_GenerateSTART(I2C1, ENABLE);
    if (!VL53L1X_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT)) goto error;

    I2C_Send7bitAddress(I2C1, VL53L1X_ADDR_DEFAULT << 1, I2C_Direction_Transmitter);
    if (!VL53L1X_WaitEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) goto error;

    I2C_SendData(I2C1, (uint8_t)(reg >> 8));
    if (!VL53L1X_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) goto error;

    I2C_SendData(I2C1, (uint8_t)(reg & 0xFFU));
    if (!VL53L1X_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) goto error;

    /* 重启，进入接收 */
    I2C_GenerateSTART(I2C1, ENABLE);
    if (!VL53L1X_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT)) goto error;

    I2C_Send7bitAddress(I2C1, VL53L1X_ADDR_DEFAULT << 1, I2C_Direction_Receiver);
    if (!VL53L1X_WaitEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) goto error;

    for (i = 0; i < len; i++) {
        if (i == (uint16_t)(len - 1U)) {
            I2C_AcknowledgeConfig(I2C1, DISABLE);
        }

        if (!VL53L1X_WaitEvent(I2C_EVENT_MASTER_BYTE_RECEIVED)) goto error;
        buf[i] = I2C_ReceiveData(I2C1);
    }

    I2C_GenerateSTOP(I2C1, ENABLE);
    I2C_AcknowledgeConfig(I2C1, ENABLE);
    return VL53L1X_OK;

error:
    VL53L1X_StopAndClear();
    return VL53L1X_ERR_I2C;
}

uint8_t VL53L1X_ReadReg8(uint16_t reg, uint8_t *value)
{
    if (value == 0) {
        return VL53L1X_ERR_NULL;
    }

    return VL53L1X_ReadMulti(reg, value, 1);
}

uint8_t VL53L1X_ReadReg16(uint16_t reg, uint16_t *value)
{
    uint8_t buf[2];
    uint8_t ret;

    if (value == 0) {
        return VL53L1X_ERR_NULL;
    }

    ret = VL53L1X_ReadMulti(reg, buf, 2);
    if (ret != VL53L1X_OK) {
        return ret;
    }

    *value = VL53L1X_MakeU16BE(buf);
    return VL53L1X_OK;
}

uint8_t VL53L1X_ReadReg32(uint16_t reg, uint32_t *value)
{
    uint8_t buf[4];
    uint8_t ret;

    if (value == 0) {
        return VL53L1X_ERR_NULL;
    }

    ret = VL53L1X_ReadMulti(reg, buf, 4);
    if (ret != VL53L1X_OK) {
        return ret;
    }

    *value = VL53L1X_MakeU32BE(buf);
    return VL53L1X_OK;
}

uint8_t VL53L1X_WriteReg8(uint16_t reg, uint8_t value)
{
    return VL53L1X_WriteMulti(reg, &value, 1);
}

uint8_t VL53L1X_WriteReg16(uint16_t reg, uint16_t value)
{
    uint8_t buf[2];

    buf[0] = (uint8_t)(value >> 8);
    buf[1] = (uint8_t)(value & 0xFFU);

    return VL53L1X_WriteMulti(reg, buf, 2);
}

uint8_t VL53L1X_WriteReg32(uint16_t reg, uint32_t value)
{
    uint8_t buf[4];

    buf[0] = (uint8_t)(value >> 24);
    buf[1] = (uint8_t)(value >> 16);
    buf[2] = (uint8_t)(value >> 8);
    buf[3] = (uint8_t)(value & 0xFFU);

    return VL53L1X_WriteMulti(reg, buf, 4);
}

/* ==================== 连通性检测 ==================== */
uint8_t VL53L1X_Detect(vl53l1x_id_t *id)
{
    uint8_t buf[2];
    uint8_t ret;

    ret = VL53L1X_ReadMulti(VL53L1X_REG_IDENTIFICATION_MODEL_ID, buf, 2);
    if (ret != VL53L1X_OK) {
        g_vl53_id.address = VL53L1X_ADDR_DEFAULT;
        g_vl53_id.model_id = 0;
        g_vl53_id.module_type = 0;
        g_vl53_id.present = 0;

        if (id != 0) {
            *id = g_vl53_id;
        }
        return ret;
    }

    g_vl53_id.address = VL53L1X_ADDR_DEFAULT;
    g_vl53_id.model_id = buf[0];
    g_vl53_id.module_type = buf[1];
    g_vl53_id.present = (buf[0] == VL53L1X_EXPECTED_MODEL_ID) ? 1U : 0U;

    if (id != 0) {
        *id = g_vl53_id;
    }

    if (!g_vl53_id.present) {
        return VL53L1X_ERR_ID;
    }

    return VL53L1X_OK;
}

uint8_t VL53L1X_Init(void)
{
    return VL53L1X_Detect(0);
}

uint8_t VL53L1X_IsPresent(void)
{
    return g_vl53_id.present;
}

uint8_t VL53L1X_GetAddress(void)
{
    return g_vl53_id.address;
}

uint8_t VL53L1X_GetModelId(void)
{
    return g_vl53_id.model_id;
}

uint8_t VL53L1X_GetModuleType(void)
{
    return g_vl53_id.module_type;
}

const vl53l1x_id_t *VL53L1X_GetIdInfo(void)
{
    return &g_vl53_id;
}

void VL53L1X_DebugPrintDetect(void)
{
    char msg[128];
    vl53l1x_id_t id;
    uint8_t ret;

    ret = VL53L1X_Detect(&id);

    if (ret == VL53L1X_OK) {
        snprintf(msg, sizeof(msg),
                 "VL53L1X found: addr=0x%02X model=0x%02X module=0x%02X\r\n",
                 id.address, id.model_id, id.module_type);
    } else if (ret == VL53L1X_ERR_ID) {
        snprintf(msg, sizeof(msg),
                 "VL53L1X ID mismatch: addr=0x%02X model=0x%02X module=0x%02X\r\n",
                 id.address, id.model_id, id.module_type);
    } else if (ret == VL53L1X_ERR_BUSY) {
        snprintf(msg, sizeof(msg), "VL53L1X detect skipped: I2C busy\r\n");
    } else {
        snprintf(msg, sizeof(msg), "VL53L1X not found at 0x%02X, ret=%u\r\n",
                 VL53L1X_ADDR_DEFAULT, ret);
    }

    UART1_SendData_NonBlocking((uint8_t*)msg, strlen(msg));
}
