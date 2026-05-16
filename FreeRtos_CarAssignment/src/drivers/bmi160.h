#ifndef BMI160_H
#define BMI160_H

#ifdef __cplusplus
extern "C" {
#endif

#define BMI160_I2C_ADDRESS      0x69

/* Registers */
#define BMI160_REG_CHIP_ID      0x00
#define BMI160_REG_ACCEL_X_L    0x12
#define BMI160_REG_CMD          0x7E

/* Commands */
#define BMI160_CMD_SOFTRESET        0xB6
#define BMI160_CMD_ACCEL_NORMAL     0x11

/* Data structure */
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} bmi160_accel_raw_t;

typedef struct {
    float x;   // in g
    float y;
    float z;
} bmi160_accel_float_t;

/* Functions */
bool sensorBMI160Init(void);
bool sensorBMI160ReadAccel(bmi160_accel_raw_t *raw);
void sensorBMI160ConvertAccel(const bmi160_accel_raw_t *raw, bmi160_accel_float_t *g);

#ifdef __cplusplus
}
#endif
#endif