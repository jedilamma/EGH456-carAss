#include <stdbool.h>
#include <stdint.h>
#include "i2cOptDriver.h"
#include "bmi160.h"
#include "FreeRTOS.h"
#include "task.h"

static uint8_t bmi160_addr = BMI160_I2C_ADDRESS;

bool sensorBMI160Init(void)
{
    uint8_t cmd, pmu;

    // Soft reset
    cmd = BMI160_CMD_SOFTRESET;
    writeI2C_Polling(bmi160_addr, BMI160_REG_CMD, &cmd);
    vTaskDelay(pdMS_TO_TICKS(100));

    // Set Normal mode
    cmd = BMI160_CMD_ACCEL_NORMAL;
    writeI2C_Polling(bmi160_addr, BMI160_REG_CMD, &cmd);
    vTaskDelay(pdMS_TO_TICKS(80));

    // Configure accelerometer (±4g, 100 Hz)
    cmd = 0x08;                                 // ±4g range
    cmd = 0x08;                                 // ±4g range
    writeI2C_Polling(bmi160_addr, BMI160_REG_ACC_RANGE, &cmd);

    cmd = 0x28;                                 // 100 Hz, normal filter
    writeI2C_Polling(bmi160_addr, BMI160_REG_ACC_CONF, &cmd);

    // Quick verify
    readI2C_Polling(bmi160_addr, BMI160_REG_PMU_STATUS, &pmu);
    if ((pmu & 0x30) != 0x10)
        return false;   // failed

    return true;
}

bool sensorBMI160ReadAccel(bmi160_accel_raw_t *raw)
{
    uint8_t buf[6] = {0};
    if (!readBurstI2C_Polling(bmi160_addr, BMI160_REG_ACCEL_X_L, buf, 6))
        return false;

    raw->x = (int16_t)((buf[1] << 8) | buf[0]);
    raw->y = (int16_t)((buf[3] << 8) | buf[2]);
    raw->z = (int16_t)((buf[5] << 8) | buf[4]);
    return true;
}

void sensorBMI160ConvertAccel(const bmi160_accel_raw_t *raw, bmi160_accel_float_t *g)
{
    g->x = raw->x / 8192.0f;   // ±4g scaling
    g->y = raw->y / 8192.0f;
    g->z = raw->z / 8192.0f;
}