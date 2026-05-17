/**************************************************************************************************
*  Filename:       i2cOptDriver.c
*  Description:    I2C driver for OPT3001 + BMI160 
*                  SINGLE OUTER MUTEX design 
*************************************************************************************************/

#include "i2cOptDriver.h"

#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/i2c.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "utils/uartstdio.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "taskman.h"

static SemaphoreHandle_t I2CSemaphore = NULL;
extern  SemaphoreHandle_t xI2CMutex;

static volatile bool g_bI2CError = false;
static volatile uint8_t g_ui8I2CSlaveAddr;
static volatile uint8_t g_ui8I2CReg;
static volatile uint8_t *g_pui8I2CData = NULL;

typedef enum
{
    I2C_IDLE = 0,
    I2C_WRITE_REG_START,
    I2C_WRITE_DATA_CONT,
    I2C_WRITE_DATA_FINISH,
    I2C_READ_REG_SEND,
    I2C_READ_BURST_START,
    I2C_READ_BURST_FINISH
} i2c_transfer_state_t;

static volatile i2c_transfer_state_t g_eI2CState = I2C_IDLE;

// ==================== ISR (unchanged) ====================
void I2C0IntHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    I2CMasterIntClear(I2C0_BASE);

    if (I2CMasterErr(I2C0_BASE) != I2C_MASTER_ERR_NONE)
    {
        g_bI2CError = true;
        g_eI2CState = I2C_IDLE;
        xSemaphoreGiveFromISR(I2CSemaphore, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        return;
    }

    switch (g_eI2CState)
    {
        case I2C_WRITE_REG_START:
            I2CMasterDataPut(I2C0_BASE, g_pui8I2CData[0]);
            I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
            g_eI2CState = I2C_WRITE_DATA_CONT;
            break;
        case I2C_WRITE_DATA_CONT:
            I2CMasterDataPut(I2C0_BASE, g_pui8I2CData[1]);
            I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
            g_eI2CState = I2C_WRITE_DATA_FINISH;
            break;
        case I2C_WRITE_DATA_FINISH:
            g_eI2CState = I2C_IDLE;
            xSemaphoreGiveFromISR(I2CSemaphore, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            break;

        case I2C_READ_REG_SEND:
            I2CMasterSlaveAddrSet(I2C0_BASE, g_ui8I2CSlaveAddr, true);
            I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
            g_eI2CState = I2C_READ_BURST_START;
            break;
        case I2C_READ_BURST_START:
            g_pui8I2CData[0] = I2CMasterDataGet(I2C0_BASE);
            I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
            g_eI2CState = I2C_READ_BURST_FINISH;
            break;
        case I2C_READ_BURST_FINISH:
            g_pui8I2CData[1] = I2CMasterDataGet(I2C0_BASE);
            g_eI2CState = I2C_IDLE;
            xSemaphoreGiveFromISR(I2CSemaphore, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            break;

        default:
            g_eI2CState = I2C_IDLE;
            xSemaphoreGiveFromISR(I2CSemaphore, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            break;
    }
}

void i2cOptDriverInit(void)
{
    if (I2CSemaphore == NULL)
    {
        I2CSemaphore = xSemaphoreCreateBinary();
        if (I2CSemaphore == NULL) { 
            SafeUARTprintf("Failed to create I2C semaphore!\r\n"); 
            while(1); 
        }
    }

    I2CMasterIntEnable(I2C0_BASE);
    IntPrioritySet(INT_I2C0, 0xA0);
    IntEnable(INT_I2C0);

    SafeUARTprintf("I2C Driver initialized - SINGLE OUTER MUTEX mode (tasks hold bus).\r\n");
}

// ===================================================================
// INTERRUPT-DRIVEN FUNCTIONS (OPT3001) - NO INTERNAL MUTEX
// ===================================================================

bool writeI2C(uint8_t ui8Addr, uint8_t ui8Reg, uint8_t *data)
{
    // Caller MUST already hold xI2CMutex
    if (I2CSemaphore == NULL || g_eI2CState != I2C_IDLE) {
        return false;
    }

    g_bI2CError = false;
    g_ui8I2CSlaveAddr = ui8Addr;
    g_ui8I2CReg       = ui8Reg;
    g_pui8I2CData     = data;
    g_eI2CState       = I2C_WRITE_REG_START;

    I2CMasterSlaveAddrSet(I2C0_BASE, ui8Addr, false);
    I2CMasterDataPut(I2C0_BASE, ui8Reg);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);

    if (xSemaphoreTake(I2CSemaphore, pdMS_TO_TICKS(100)) == pdPASS)
    {
        return !g_bI2CError;
    }

    g_eI2CState = I2C_IDLE;
    return false;
}

bool readI2C(uint8_t ui8Addr, uint8_t ui8Reg, uint8_t *data)
{
    if (I2CSemaphore == NULL || g_eI2CState != I2C_IDLE)
        return false;

    g_bI2CError = false;
    g_ui8I2CSlaveAddr = ui8Addr;
    g_ui8I2CReg       = ui8Reg;
    g_pui8I2CData     = data;
    g_eI2CState       = I2C_READ_REG_SEND;

    I2CMasterSlaveAddrSet(I2C0_BASE, ui8Addr, false);
    I2CMasterDataPut(I2C0_BASE, ui8Reg);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);

    if (xSemaphoreTake(I2CSemaphore, pdMS_TO_TICKS(100)) == pdPASS)
    {
        return !g_bI2CError;
    }

    g_eI2CState = I2C_IDLE;
    return false;
}

bool readBurstI2C(uint8_t ui8Addr, uint8_t ui8Reg, uint8_t *data, uint8_t len)
{
    if (I2CSemaphore == NULL || g_eI2CState != I2C_IDLE || len == 0)
        return false;

    g_bI2CError = false;
    g_ui8I2CSlaveAddr = ui8Addr;
    g_pui8I2CData     = data;
    g_eI2CState       = I2C_READ_REG_SEND;

    I2CMasterSlaveAddrSet(I2C0_BASE, ui8Addr, false);
    I2CMasterDataPut(I2C0_BASE, ui8Reg);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);

    if (xSemaphoreTake(I2CSemaphore, pdMS_TO_TICKS(100)) == pdPASS)
    {
        return !g_bI2CError;
    }

    g_eI2CState = I2C_IDLE;
    return false;
}

// ===================================================================
// POLLING FUNCTIONS (BMI160) - NO INTERNAL MUTEX
// ===================================================================

bool writeI2C_Polling(uint8_t ui8Addr, uint8_t ui8Reg, uint8_t *data)
{
    while (I2CMasterBusy(I2C0_BASE));

    I2CMasterSlaveAddrSet(I2C0_BASE, ui8Addr, false);
    I2CMasterDataPut(I2C0_BASE, ui8Reg);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    while (I2CMasterBusy(I2C0_BASE));

    if (I2CMasterErr(I2C0_BASE) != I2C_MASTER_ERR_NONE)
        return false;

    I2CMasterDataPut(I2C0_BASE, *data);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
    while (I2CMasterBusy(I2C0_BASE));

    return (I2CMasterErr(I2C0_BASE) == I2C_MASTER_ERR_NONE);
}

bool readI2C_Polling(uint8_t ui8Addr, uint8_t ui8Reg, uint8_t *data)
{
    uint8_t dummy;
    if (!readBurstI2C_Polling(ui8Addr, ui8Reg, &dummy, 1))
        return false;
    *data = dummy;
    return true;
}

bool readBurstI2C_Polling(uint8_t ui8Addr, uint8_t ui8Reg, uint8_t *data, uint8_t len)
{
    if (len == 0)
        return false;

    while (I2CMasterBusy(I2C0_BASE));

    I2CMasterSlaveAddrSet(I2C0_BASE, ui8Addr, false);
    I2CMasterDataPut(I2C0_BASE, ui8Reg);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);
    while (I2CMasterBusy(I2C0_BASE));

    if (I2CMasterErr(I2C0_BASE) != I2C_MASTER_ERR_NONE)
        return false;

    I2CMasterSlaveAddrSet(I2C0_BASE, ui8Addr, true);

    if (len == 1)
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
    else
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);

    while (I2CMasterBusy(I2C0_BASE));

    for (uint8_t i = 0; i < len; i++)
    {
        data[i] = I2CMasterDataGet(I2C0_BASE);
        if (i < len-1)
            I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
        else
            I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
        while (I2CMasterBusy(I2C0_BASE));
    }

    return (I2CMasterErr(I2C0_BASE) == I2C_MASTER_ERR_NONE);
}