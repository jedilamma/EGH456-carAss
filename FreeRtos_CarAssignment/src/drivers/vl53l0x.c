#include <stdbool.h>
#include <stdint.h>
#include "i2cOptDriver.h"          // <-- Use the shared driver
#include "vl53l0x.h"
#include "FreeRTOS.h"
#include "task.h"
#include "utils/uartstdio.h"
#include "taskman.h"               // for SafeUARTprintf

// VL53L0X is on I2C2 (same as BMI160 + OPT3001)
#define VL53_I2C_ADDR   VL53L0X_I2C_ADDRESS   // 0x29

bool sensorVL53L0XInit(void)
{
    uint8_t id = 0;

    SafeUARTprintf("[VL53] Initializing (using shared I2C2 driver)...\r\n");

    // Do NOT call VL53_I2C2Init() here — I2C2 is already configured in prvConfigurei2c()

    // Try to read Model ID a few times (some modules are slow to respond after reset)
    for (int i = 0; i < 5; i++)
    {
        if (readI2C_Polling(VL53_I2C_ADDR, 0xC0, &id))
        {
            SafeUARTprintf("[VL53] Model ID = 0x%02X\r\n", id);
            if (id == 0xEE || id == 0xE1)   // Common responses on cheap modules
            {
                SafeUARTprintf("[VL53] Sensor detected!\r\n");
                return true;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    SafeUARTprintf("[VL53] ERROR: Could not detect VL53L0X\r\n");
    return false;
}

bool VL53_TestSingleShot(uint16_t *distance_mm)
{
    uint8_t status = 0;
    uint8_t buf[2];

    // Start single-shot measurement
    uint8_t cmd = 0x01;
    if (!writeI2C_Polling(VL53_I2C_ADDR, 0x00, &cmd))
        return false;

    // Wait for measurement to complete (up to ~200ms)
    for (int i = 0; i < 20; i++)
    {
        vTaskDelay(pdMS_TO_TICKS(10));
        if (readI2C_Polling(VL53_I2C_ADDR, 0x13, &status) && (status & 0x01))
            break;
    }

    // Read distance (Result Range Status + distance)
    if (!readBurstI2C_Polling(VL53_I2C_ADDR, 0x1E, buf, 2))
        return false;

    *distance_mm = ((uint16_t)buf[0] << 8) | buf[1];

    // Clear interrupt
    cmd = 0x01;
    writeI2C_Polling(VL53_I2C_ADDR, 0x13, &cmd);

    return true;
}