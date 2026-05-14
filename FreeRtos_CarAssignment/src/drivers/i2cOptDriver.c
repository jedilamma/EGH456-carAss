/**************************************************************************************************
*  Filename:       i2cOptDriver.c
*  By:             Jesse Haviland
*  Created:        1 February 2019
*  Revised:        23 March 2019
*  Revision:       2.0
*
*  Description:    i2c Driver for use with opt3001.c and the TI OP3001 Optical Sensor
*************************************************************************************************/

// ----------------------- Includes -----------------------
#include "i2cOptDriver.h"

#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"          // Needed for INT_I2C0
#include "driverlib/i2c.h"        // I2CMasterInitExpClk, I2CMasterIntEnable, etc.
#include "driverlib/interrupt.h"  // IntPrioritySet, IntEnable, etc.
#include "driverlib/sysctl.h"     // If you use SysCtlClockGet
#include "utils/uartstdio.h"
#include "FreeRTOS.h"
#include "semphr.h"

static SemaphoreHandle_t I2CSemaphore = NULL;

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
        /* ============== WRITE SEQUENCE ============== */
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

        /* ============== READ SEQUENCE ============== */
        case I2C_READ_REG_SEND:
            // After sending register address (single send), we must do a repeated start + read
            I2CMasterSlaveAddrSet(I2C0_BASE, g_ui8I2CSlaveAddr, true);   // read direction
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


/* ----------------------- Init (call once) ----------------------- */
void i2cOptDriverInit(void)
{
    if (I2CSemaphore == NULL)
    {
        I2CSemaphore = xSemaphoreCreateBinary();
        if (I2CSemaphore == NULL)
        {
            UARTprintf("Failed to create I2C semaphore!\n");
            while(1);
        }
    }

    I2CMasterIntEnable(I2C0_BASE);
    IntPrioritySet(INT_I2C0, 0xA0);   // safe priority
    IntEnable(INT_I2C0);
}

//------------------------------------------------------
// interrupt driven write command for I2C and OPT3001
//------------------------------------------------------
bool writeI2C(uint8_t ui8Addr, uint8_t ui8Reg, uint8_t *data)
{
    if (I2CSemaphore == NULL || g_eI2CState != I2C_IDLE)
        return false;

    g_bI2CError = false;
    g_ui8I2CSlaveAddr = ui8Addr;
    g_ui8I2CReg       = ui8Reg;
    g_pui8I2CData     = data;
    g_eI2CState       = I2C_WRITE_REG_START;

    /* Start the transfer (task context) */
    I2CMasterSlaveAddrSet(I2C0_BASE, ui8Addr, false);
    I2CMasterDataPut(I2C0_BASE, ui8Reg);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);

    /* Block until ISR completes or timeout */
    if (xSemaphoreTake(I2CSemaphore, pdMS_TO_TICKS(100)) == pdPASS)
    {
        //UARTprintf("readI2C completed, error=%d\n", g_bI2CError);   // DEBUG
        return !g_bI2CError;
    }

    /* Timeout */
    UARTprintf("readI2C TIMEOUT!\n");   // DEBUG
    g_eI2CState = I2C_IDLE;

    return false;
}

//------------------------------------------------------
// interrupt driven read command for I2C and OPT3001
//------------------------------------------------------
bool readI2C(uint8_t ui8Addr, uint8_t ui8Reg, uint8_t *data)
{
    if (I2CSemaphore == NULL || g_eI2CState != I2C_IDLE)
        return false;

    g_bI2CError = false;
    g_ui8I2CSlaveAddr = ui8Addr;
    g_ui8I2CReg       = ui8Reg;
    g_pui8I2CData     = data;
    g_eI2CState       = I2C_READ_REG_SEND;

    /* Start the transfer (task context) */
    I2CMasterSlaveAddrSet(I2C0_BASE, ui8Addr, false);
    I2CMasterDataPut(I2C0_BASE, ui8Reg);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);

    /* Block until ISR completes or timeout */
    if (xSemaphoreTake(I2CSemaphore, pdMS_TO_TICKS(100)) == pdPASS)
    {
        //UARTprintf("readI2C completed, error=%d\n", g_bI2CError);   // DEBUG
        return !g_bI2CError;
    }

    /* Timeout */
    UARTprintf("readI2C TIMEOUT!\n");   // DEBUG
    g_eI2CState = I2C_IDLE;
    return false;
}


