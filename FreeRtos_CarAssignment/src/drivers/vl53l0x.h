#ifndef VL53L0X_H
#define VL53L0X_H

#include <stdbool.h>
#include <stdint.h>

/*
 * VL53L0X driver using the shared I2C2 polling functions
 * (same style as BMI160 + OPT3001).
 *
 * I2C2 must already be initialized by prvConfigurei2c() before using this driver.
 */

#define VL53L0X_I2C_ADDRESS     0x29

/**
 * @brief Initialize and detect the VL53L0X sensor.
 * @return true if sensor was detected, false otherwise.
 */
bool sensorVL53L0XInit(void);

/**
 * @brief Perform a single-shot distance measurement.
 * @param distance_mm Pointer to store the measured distance in millimeters.
 * @return true if measurement was successful, false on I2C error.
 */
bool VL53_TestSingleShot(uint16_t *distance_mm);

#endif /* VL53L0X_H */