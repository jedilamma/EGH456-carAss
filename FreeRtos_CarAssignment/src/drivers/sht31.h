#ifndef SHT31_H_
#define SHT31_H_

#include <stdint.h>
#include <stdbool.h>

/*
 * SHT31 temperature and humidity driver for the EGH456 hardware stack.
 *
 * Reference hardware route:
 *   BoosterPack 2 I2C bus through the QUT I2C breakout board
 *   SCL = PN5 / I2C2SCL
 *   SDA = PN4 / I2C2SDA
 *
 * Grove SHT31 connector order:
 *   pin 1 = SCL
 *   pin 2 = SDA
 *   pin 3 = VCC, use 3V3
 *   pin 4 = GND
 *
 * This driver only configures I2C2 on PN5/PN4. It does not scan or reset
 * other LaunchPad routes, so it will not disturb BP1 motor-control pins,
 * display pins, ADC0 touch screen inputs, or Timer 1.
 */
typedef enum
{
    SHT31_STATUS_OK = 0,
    SHT31_STATUS_NULL_POINTER,
    SHT31_STATUS_SENSOR_NOT_FOUND,
    SHT31_STATUS_I2C_TIMEOUT,
    SHT31_STATUS_I2C_COMMAND_FAILED,
    SHT31_STATUS_I2C_READ_FAILED,
    SHT31_STATUS_TEMPERATURE_CRC_FAILED,
    SHT31_STATUS_HUMIDITY_CRC_FAILED,
    SHT31_STATUS_STATUS_CRC_FAILED
} SHT31_Status_t;

typedef struct
{
    float temperature_c;
    float humidity_rh;
    uint16_t raw_temperature;
    uint16_t raw_humidity;
    bool valid;
} SHT31_Reading_t;

void SHT31_InitBus(uint32_t system_clock_hz);
bool SHT31_IsDetected(void);
uint8_t SHT31_GetI2CAddress(void);
const char *SHT31_GetPinRouteDescription(void);
SHT31_Status_t SHT31_Detect(void);
SHT31_Status_t SHT31_ReadBlocking(SHT31_Reading_t *reading);
const char *SHT31_StatusToString(SHT31_Status_t status);

#endif /* SHT31_H_ */
