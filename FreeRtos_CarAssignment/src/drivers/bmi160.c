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

    cmd = BMI160_CMD_SOFTRESET;
    writeI2C_Polling(bmi160_addr, BMI160_REG_CMD, &cmd);
    vTaskDelay(pdMS_TO_TICKS(100));

    cmd = BMI160_CMD_ACCEL_NORMAL;
    writeI2C_Polling(bmi160_addr, BMI160_REG_CMD, &cmd);
    vTaskDelay(pdMS_TO_TICKS(80));

    cmd = 0x08;                     // ±8g (less sensitive)
    writeI2C_Polling(bmi160_addr, BMI160_REG_ACC_RANGE, &cmd);

    cmd = 0x28;
    writeI2C_Polling(bmi160_addr, BMI160_REG_ACC_CONF, &cmd);

    readI2C_Polling(bmi160_addr, BMI160_REG_PMU_STATUS, &pmu);
    return ((pmu & 0x30) == 0x10);
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
    // ±8g range = 4096 LSB/g
    g->x = raw->x / 4096;
    g->y = raw->y / 4096;
    g->z = raw->z / 4096;
}