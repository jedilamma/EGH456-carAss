#include <stdbool.h>
#include <stdint.h>
#include "i2cOptDriver.h"
#include "bmi160.h"
#include "utils/uartstdio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "system_utils.h"

bool sensorBMI160Init(void)
{
    uint8_t cmd = 0;
    uint8_t pmu = 0;

    SafeUARTprintf("BMI160 Init START (Addr=0x69)\r\n");

    // Soft Reset
    cmd = BMI160_CMD_SOFTRESET;
    writeI2CByte(BMI160_I2C_ADDRESS, BMI160_REG_CMD, cmd);
    vTaskDelay(pdMS_TO_TICKS(250));

    // Set ±2g range FIRST
    cmd = 0x03;   // 0x03 = ±2g
    writeI2CByte(BMI160_I2C_ADDRESS, 0x41, cmd);   // ACC_RANGE
    vTaskDelay(pdMS_TO_TICKS(50));

    // Enable Normal Mode - Try TWICE (common workaround)
    cmd = 0x11;
    writeI2CByte(BMI160_I2C_ADDRESS, BMI160_REG_CMD, cmd);
    vTaskDelay(pdMS_TO_TICKS(100));

    writeI2CByte(BMI160_I2C_ADDRESS, BMI160_REG_CMD, cmd);  // Second write helps
    vTaskDelay(pdMS_TO_TICKS(150));

    // Check status
    readI2C(BMI160_I2C_ADDRESS, 0x03, &pmu);
    SafeUARTprintf("PMU Status = 0x%02X\r\n", pmu);

    if ((pmu & 0x30) == 0x10)
        SafeUARTprintf(">>> SUCCESS: Accelerometer ACTIVE <<<\r\n");
    else
        SafeUARTprintf("Still in suspend mode!\r\n");

    return true;
}

bool sensorBMI160ReadAccel(bmi160_accel_raw_t *raw)
{
    uint8_t buf[6] = {0};

    if (!readBurstI2CPolling(BMI160_I2C_ADDRESS, BMI160_REG_ACCEL_X_L, buf, 6))
    {
        return false;
    }

    raw->x = (int16_t)((buf[1] << 8) | buf[0]);
    raw->y = (int16_t)((buf[3] << 8) | buf[2]);
    raw->z = (int16_t)((buf[5] << 8) | buf[4]);

    // Quick sanity check
    if (raw->x == 0 && raw->y == 0 && raw->z == 0)
        return false;   // still zero → treat as bad read

    return true;
}

void sensorBMI160ConvertAccel(const bmi160_accel_raw_t *raw, bmi160_accel_float_t *g)
{
    // ±2g range by default (sensitivity = 16384 LSB/g)
    g->x = raw->x / 16384.0f;
    g->y = raw->y / 16384.0f;
    g->z = raw->z / 16384.0f;
}