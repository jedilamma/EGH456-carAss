/**************************************************************************************************
*  Filename:       opt3001.h
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
#ifndef OPT3001_H
#define OPT3001_H

#ifdef __cplusplus
extern "C"
{
#endif

/* ------------------------------------------------------------------------------------------------
 *                                           Constants
 * ------------------------------------------------------------------------------------------------
 */

/* Slave address */
#define OPT3001_I2C_ADDRESS             0x47

/* Register addresses */
#define REG_RESULT                      0x00
#define REG_CONFIGURATION               0x01
#define REG_LOW_LIMIT                   0x02
#define REG_HIGH_LIMIT                  0x03

#define REG_MANUFACTURER_ID             0x7E
#define REG_DEVICE_ID                   0x7F

/* Register values */
#define MANUFACTURER_ID                 0x5449  // ID check = TI
#define DEVICE_ID                       0x3001  // Device ID = 3001

#define CONFIG_RESET                    0xC810                   
#define CONFIG_TEST                     0xCC10

#define CONFIG_ENABLE                   0x10C4 // stored little-endian: bytes sent on wire are [0xC4, 0x10], so sensor register receives 0xC410 (continuous conversion, 800ms)
#define CONFIG_DISABLE                  0x10C0 // stored little-endian: bytes sent on wire are [0xC0, 0x10], so sensor register receives 0xC010 (shutdown)

/* Bit values */
#define DATA_RDY_BIT                    0x0080  // Data ready

/* Register length */
#define REGISTER_LENGTH                 2

/* Sensor data size */
#define DATA_LENGTH                     2

/*********************************************************************
 * FUNCTIONS
 */
extern bool sensorOpt3001Init(void);
extern bool sensorOpt3001Enable(bool enable);
extern bool sensorOpt3001Read(uint16_t *rawData);
extern void sensorOpt3001Convert(uint16_t rawData, float *convertedLux);
extern bool sensorOpt3001Test(void);

#ifdef __cplusplus
}
#endif

#endif /* OPT3001_H */
