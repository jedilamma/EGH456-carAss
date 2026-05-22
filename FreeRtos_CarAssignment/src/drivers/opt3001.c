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
#include "system_utils.h"


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
    uint8_t data[2];

    SafeUARTprintf("[OPT] Soft Reset...\r\n");
    data[0] = 0xC4;  // MSB
    data[1] = 0x10;  // LSB
    if (!writeI2C_Polling(OPT3001_I2C_ADDRESS, REG_CONFIGURATION, data))
    {
        SafeUARTprintf("[OPT] Reset FAILED\r\n");
        return false;
    }

    vTaskDelay(pdMS_TO_TICKS(150));

    if (!sensorOpt3001Enable(true))
        return false;

    SafeUARTprintf("[OPT] Waiting for first conversion (800ms mode)...\r\n");
    vTaskDelay(pdMS_TO_TICKS(1000));     // ← Add this (1 second)

    return true;
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
    uint16_t cfg = enable ? CONFIG_ENABLE : CONFIG_DISABLE;
    uint8_t data[2];
    uint8_t rx[2] = {0};

    data[0] = (uint8_t)(cfg >> 8);
    data[1] = (uint8_t)(cfg & 0xFF);

    SafeUARTprintf("[OPT] Writing config 0x%04X using interrupt path...\r\n", cfg);

    if (!writeI2C(OPT3001_I2C_ADDRESS, REG_CONFIGURATION, data))
    {
        SafeUARTprintf("[OPT] Config write FAILED\r\n");
        return false;
    }

    vTaskDelay(pdMS_TO_TICKS(80));

    // === Proper 2-byte read ===
    if (!readBurstI2C_Polling(OPT3001_I2C_ADDRESS, REG_CONFIGURATION, rx, 2))
    {
        SafeUARTprintf("[OPT] 2-byte config read FAILED\r\n");
        return false;
    }

    uint16_t readback = ((uint16_t)rx[0] << 8) | rx[1];   // Big-endian from sensor

    SafeUARTprintf("[OPT] Readback = 0x%04X (expected 0x%04X)\r\n", readback, cfg);

    if (readback != cfg)
    {
        SafeUARTprintf("[OPT] Still mismatch!\r\n");
        return false;
    }

    vTaskDelay(pdMS_TO_TICKS(400));
    SafeUARTprintf("[OPT] Continuous mode successfully enabled!\r\n");
    return true;
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
// bool sensorOpt3001Read(uint16_t *rawData)
// {
// 	bool data_ready;
// 	uint16_t val;

// 	// Read configuration register to check if a conversion result is ready
// 	if (!readI2C(OPT3001_I2C_ADDRESS, REG_CONFIGURATION, (uint8_t *)&val))
// 	{
// 		return false;
// 	}

// 	// Sensor sends MSByte first; swap bytes after storing into little-endian val
// 	val = (val >> 8) | (val << 8);

// 	// DATA_RDY is bit 7 of the LSB of the OPT3001 configuration register
// 	data_ready = (val & DATA_RDY_BIT) != 0;

// 	if (!data_ready)
// 	{
// 		return false;
// 	}

// 	// Conversion complete — read the result register
// 	if (!readI2C(OPT3001_I2C_ADDRESS, REG_RESULT, (uint8_t *)&val))
// 	{
// 		return false;
// 	}

// 	// Swap bytes (sensor sends MSByte first, MCU stores little-endian)
// 	*rawData = (val >> 8) | (val << 8);

// 	return true;
// }
bool sensorOpt3001Read(uint16_t *rawData)
{
    uint16_t val = 0;

    if (rawData == NULL)
        return false;

    // In Continuous mode, just read the Result register directly.
    // No need to check the Config register every time.
    if (!readI2C_Polling(OPT3001_I2C_ADDRESS, REG_RESULT, (uint8_t*)&val))
    {
        return false;
    }

    // Byte swap (sensor sends MSB first)
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
