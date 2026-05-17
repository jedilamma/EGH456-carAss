/**************************************************************************************************
*  Filename:       i2cOptDriver.h
*  By:             Jesse Haviland
*  Created:        1 February 2019
*  Revised:        2 February 2019
*  Revision:       1.0
*
*  Description:    i2c Driver for use with the Texas Instruments OP3001 Optical Sensor

(Refactored for FreeRTOS interrupt-driven I2C)
*************************************************************************************************/
#include <stdbool.h>
#include <stdint.h>


#ifndef _I2COPTDRIVER_H_
#define _I2COPTDRIVER_H_
extern bool writeI2CByte(uint8_t ui8Addr, uint8_t ui8Reg, uint8_t data);
extern bool readBurstI2CPolling(uint8_t ui8Addr, uint8_t ui8Reg, uint8_t *data, uint8_t len);



// ----------------------- Includes -----------------------
#include <stdbool.h>
#include <stdint.h>



// ----------------------- Exported prototypes -----------------------
extern bool writeI2C(uint8_t ui8Addr, uint8_t ui8Reg, uint8_t *Data);
extern bool readI2C(uint8_t ui8Addr, uint8_t ui8Reg, uint8_t *Data);
//extern bool writeI2C_Polling(uint8_t ui8Addr, uint8_t ui8Reg, uint8_t *Data);
//extern bool readI2C_Polling(uint8_t ui8Addr, uint8_t ui8Reg, uint8_t *Data);
extern bool readBurstI2C(uint8_t ui8Addr, uint8_t ui8Reg, uint8_t *data, uint8_t len);

/* Call this once after I2CMasterInitExpClk() */
extern void i2cOptDriverInit(void);

#endif /* _I2COPTDRIVER_H_ */
