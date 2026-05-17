#ifndef BMI160_H
#define BMI160_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#define BMI160_I2C_ADDRESS      0x69

/* Registers */
#define BMI160_REG_ACCEL_X_L    0x12
#define BMI160_REG_ACC_CONF     0x40
#define BMI160_REG_ACC_RANGE    0x41
#define BMI160_REG_PMU_STATUS   0x03
#define BMI160_REG_CMD          0x7E

/* Commands */
#define BMI160_CMD_SOFTRESET    0xB6
#define BMI160_CMD_ACCEL_NORMAL 0x11

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} bmi160_accel_raw_t;

typedef struct {
    int16_t x;   // in g
    int16_t y;
    int16_t z;
} bmi160_accel_uint_t;

// Add near the top with the other globals
#define PRINT_BUFFER_SIZE   256

extern char g_pcPrintBuffer[PRINT_BUFFER_SIZE];

/* Functions */
bool sensorBMI160Init(void);
bool sensorBMI160ReadAccel(bmi160_accel_raw_t *raw);
void sensorBMI160ConvertAccel(const bmi160_accel_raw_t *raw, bmi160_accel_uint_t *g);

#ifdef __cplusplus
}
#endif
#endif