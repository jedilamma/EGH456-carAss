#ifndef _I2COPTDRIVER_H_
#define _I2COPTDRIVER_H_

#include <stdbool.h>
#include <stdint.h>

extern bool writeI2C(uint8_t ui8Addr, uint8_t ui8Reg, uint8_t *Data);
extern bool readI2C(uint8_t ui8Addr, uint8_t ui8Reg, uint8_t *Data);
extern bool readBurstI2C(uint8_t ui8Addr, uint8_t ui8Reg, uint8_t *data, uint8_t len);

// === NEW POLLING FUNCTIONS (added for BMI160) ===
extern bool writeI2C_Polling(uint8_t ui8Addr, uint8_t ui8Reg, uint8_t *Data);
extern bool readI2C_Polling(uint8_t ui8Addr, uint8_t ui8Reg, uint8_t *Data);
extern bool readBurstI2C_Polling(uint8_t ui8Addr, uint8_t ui8Reg, uint8_t *data, uint8_t len);

extern void i2cOptDriverInit(void);

#endif