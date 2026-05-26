
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include "i2cOptDriver.h"
#include "opt3001.h"
#include "utils/uartstdio.h"
#include "system_utils.h"

// ====================== SHT31 on I2C2 (shared bus) ======================
#define SHT31_I2C_BASE          I2C2_BASE
#define SHT31_ADDR              0x44        // Default address (some boards use 0x45)

static bool SHT31_WriteCommand(uint16_t command)
{
    // Write 2-byte command (MSB first)
    I2CMasterSlaveAddrSet(SHT31_I2C_BASE, SHT31_ADDR, false);
    I2CMasterDataPut(SHT31_I2C_BASE, (command >> 8) & 0xFF);
    I2CMasterControl(SHT31_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    while (I2CMasterBusy(SHT31_I2C_BASE)) {}

    I2CMasterDataPut(SHT31_I2C_BASE, command & 0xFF);
    I2CMasterControl(SHT31_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
    while (I2CMasterBusy(SHT31_I2C_BASE)) {}

    return (I2CMasterErr(SHT31_I2C_BASE) == I2C_MASTER_ERR_NONE);
}

static bool SHT31_Read6Bytes(uint8_t *data)
{
    I2CMasterSlaveAddrSet(SHT31_I2C_BASE, SHT31_ADDR, true);
    I2CMasterControl(SHT31_I2C_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
    while (I2CMasterBusy(SHT31_I2C_BASE)) {}
    data[0] = I2CMasterDataGet(SHT31_I2C_BASE);

    for (int i = 1; i < 5; i++) {
        I2CMasterControl(SHT31_I2C_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
        while (I2CMasterBusy(SHT31_I2C_BASE)) {}
        data[i] = I2CMasterDataGet(SHT31_I2C_BASE);
    }

    I2CMasterControl(SHT31_I2C_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
    while (I2CMasterBusy(SHT31_I2C_BASE)) {}
    data[5] = I2CMasterDataGet(SHT31_I2C_BASE);

    return (I2CMasterErr(SHT31_I2C_BASE) == I2C_MASTER_ERR_NONE);
}

bool SHT31_Init(void)
{
    // Just check if bus is alive (optional)
    return true;
}

// Single-shot read with clock stretching disabled (command 0x2400 = high repeatability)
bool SHT31_Read(float *temp_c, float *humidity_rh)
{
    uint8_t rx[6] = {0};
    uint16_t rawTemp, rawHum;

    if (!SHT31_WriteCommand(0x2400)) return false;

    // SHT31 needs ~15-20ms for measurement
    vTaskDelay(pdMS_TO_TICKS(20));

    if (!SHT31_Read6Bytes(rx)) return false;

    rawTemp = (rx[0] << 8) | rx[1];
    rawHum  = (rx[3] << 8) | rx[4];

    *temp_c     = -45.0f + (175.0f * (float)rawTemp  / 65535.0f);
    *humidity_rh = 100.0f * (float)rawHum / 65535.0f;

    return true;
}