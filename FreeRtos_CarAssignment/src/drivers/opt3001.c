/**************************************************************************************************
 *  Filename:       opt3001.c
 *  Revised:        
 *  Revision:       
 *
 *  Description:    Driver for the Texas Instruments OP3001 Optical Sensor
 *
 *  Copyright (C) 2014 - 2015 Texas Instruments Incorporated - http://www.ti.com/
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *************************************************************************************************/

/* ------------------------------------------------------------------------------------------------
 *                                          Includes
 * ------------------------------------------------------------------------------------------------
 */
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include "i2cOptDriver.h"
#include "opt3001.h"
#include "utils/uartstdio.h"



/* ------------------------------------------------------------------------------------------------
 *                                           Local Functions
 * ------------------------------------------------------------------------------------------------
 */


/* ------------------------------------------------------------------------------------------------
 *                                           Local Variables
 * ------------------------------------------------------------------------------------------------
 */

/* ------------------------------------------------------------------------------------------------
 *                                           Public functions
 * -------------------------------------------------------------------------------------------------
 */


/**************************************************************************************************
 * @fn          sensorOpt3001Init
 *
 * @brief       Initialize the temperature sensor by reseting the sensor
 *
 * @return      none
 **************************************************************************************************/
bool sensorOpt3001Init(void)
{
	//Disable the sensor
	if (!sensorOpt3001Enable(false))
	{
		return false;
	}

	//Enable the sensor
	return sensorOpt3001Enable(true);
}


/**************************************************************************************************
 * @fn          sensorOpt3001Enable
 *
 * @brief       Turn the sensor on or off
 *
 * @return      none
 **************************************************************************************************/
bool sensorOpt3001Enable(bool enable)
{
	uint16_t val;

	if (enable)
	{
		val = CONFIG_ENABLE;
	}
	else
	{
		val = CONFIG_DISABLE;
	}

	return writeI2C(OPT3001_I2C_ADDRESS, REG_CONFIGURATION, (uint8_t*)&val);
}


/**************************************************************************************************
 * @fn          sensorOpt3001Read
 *
 * @brief       Read the result register
 *
 * @param       Buffer to store data in
 *
 * @return      TRUE if valid data
 **************************************************************************************************/
bool sensorOpt3001Read(uint16_t *rawData)
{
	bool data_ready;
	uint16_t val;

	// Read configuration register to check if a conversion result is ready
	if (!readI2C(OPT3001_I2C_ADDRESS, REG_CONFIGURATION, (uint8_t *)&val))
	{
		return false;
	}

	// Sensor sends MSByte first; swap bytes after storing into little-endian val
	val = (val >> 8) | (val << 8);

	// DATA_RDY is bit 7 of the LSB of the OPT3001 configuration register
	data_ready = (val & DATA_RDY_BIT) != 0;

	if (!data_ready)
	{
		return false;
	}

	// Conversion complete — read the result register
	if (!readI2C(OPT3001_I2C_ADDRESS, REG_RESULT, (uint8_t *)&val))
	{
		return false;
	}

	// Swap bytes (sensor sends MSByte first, MCU stores little-endian)
	*rawData = (val >> 8) | (val << 8);

	return true;
}


/**************************************************************************************************
 * @fn          sensorOpt3001Test
 *
 * @brief       Run a sensor self-test
 *
 * @return      TRUE if passed, FALSE if failed
 **************************************************************************************************/
bool sensorOpt3001Test(void)
{
	uint16_t val;

	// Check manufacturer ID
	if (!readI2C(OPT3001_I2C_ADDRESS, REG_MANUFACTURER_ID, (uint8_t *)&val))
	{
		return false;
	}

	// Swap bytes (sensor sends MSByte first, MCU stores little-endian)
	val = (val >> 8) | (val << 8);

	if (val != MANUFACTURER_ID)
	{
		return false;
	}

	UARTprintf("Manufacturer ID Correct: %c%c\n", (val >> 8) & 0x00FF, val & 0x00FF);

	// Check device ID
	if (!readI2C(OPT3001_I2C_ADDRESS, REG_DEVICE_ID, (uint8_t *)&val))
	{
		return false;
	}

	// Swap bytes (sensor sends MSByte first, MCU stores little-endian)
	val = (val >> 8) | (val << 8);

	if (val != DEVICE_ID)
	{
		return false;
	}

	UARTprintf("Device ID Correct: %02x%02x\n", (val >> 8) & 0x00FF, val & 0x00FF);

	return true;
}

/**************************************************************************************************
 * @fn          sensorOpt3001Convert
 *
 * @brief       Convert raw data to object and ambience temperature
 *
 * @param       rawData - raw data from sensor
 *
 * @param       convertedLux - converted value (lux)
 *
 * @return      none
 **************************************************************************************************/
void sensorOpt3001Convert(uint16_t rawData, float *convertedLux)
{
    uint16_t exponent;
    uint16_t mantissa;

    /* Extract exponent (bits 15:12) and mantissa (bits 11:0) */
    exponent = (rawData >> 12) & 0x0F;
    mantissa = rawData & 0x0FFF;

    /* Correct conversion according to OPT3001 datasheet */
    *convertedLux = (float)mantissa * 0.01f * (1 << exponent);
}
