#include "sht31.h"

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/i2c.h"

#define SHT31_I2C_ADDRESS_LOW           0x44U
#define SHT31_I2C_ADDRESS_HIGH          0x45U

#define SHT31_COMMAND_MEAS_HIGH_MSB     0x24U
#define SHT31_COMMAND_MEAS_HIGH_LSB     0x00U
#define SHT31_COMMAND_READ_STATUS_MSB   0xF3U
#define SHT31_COMMAND_READ_STATUS_LSB   0x2DU

#define SHT31_READ_LENGTH               6U
#define SHT31_STATUS_READ_LENGTH        3U
#define SHT31_MEASUREMENT_DELAY_MS      20U
#define SHT31_I2C_WAIT_TIMEOUT          1000000U

typedef struct
{
    const char *description;
    uint32_t i2c_base;
    uint32_t i2c_periph;
    uint32_t gpio_periph;
    uint32_t gpio_base;
    uint32_t scl_pin;
    uint32_t sda_pin;
    uint32_t scl_config;
    uint32_t sda_config;
} SHT31_I2CRoute_t;

static const SHT31_I2CRoute_t g_route =
{
    "BP2/QUT I2C breakout: SCL=PN5/I2C2SCL, SDA=PN4/I2C2SDA",
    I2C2_BASE,
    SYSCTL_PERIPH_I2C2,
    SYSCTL_PERIPH_GPION,
    GPIO_PORTN_BASE,
    GPIO_PIN_5,
    GPIO_PIN_4,
    GPIO_PN5_I2C2SCL,
    GPIO_PN4_I2C2SDA
};

static uint32_t g_sht31_system_clock_hz = 120000000U;
static uint8_t g_sht31_i2c_address = 0U;
static bool g_sht31_detected = false;

static void SHT31_DelayMs(uint32_t delay_ms)
{
    uint32_t loops = (g_sht31_system_clock_hz / 3U / 1000U) * delay_ms;
    SysCtlDelay(loops);
}

static void SHT31_ForceI2CPullups(void)
{
    /*
     * The Grove SHT31 board already has 10k pull-ups to VCC on SCL and SDA.
     * The weak internal pull-ups are still enabled as a safe fallback for the
     * QUT breakout wiring. This only affects PN5/PN4, the shared BP2 I2C bus.
     */
    HWREG(g_route.gpio_base + GPIO_O_PUR) |= (g_route.scl_pin | g_route.sda_pin);
}

static void SHT31_ResetI2C2(void)
{
    SysCtlPeripheralReset(g_route.i2c_periph);
    SHT31_DelayMs(2U);

    while (!SysCtlPeripheralReady(g_route.i2c_periph))
    {
    }

    GPIOPinConfigure(g_route.scl_config);
    GPIOPinConfigure(g_route.sda_config);
    GPIOPinTypeI2CSCL(g_route.gpio_base, g_route.scl_pin);
    GPIOPinTypeI2C(g_route.gpio_base, g_route.sda_pin);
    SHT31_ForceI2CPullups();

    I2CMasterInitExpClk(g_route.i2c_base, g_sht31_system_clock_hz, false);
    I2CMasterEnable(g_route.i2c_base);
}

static SHT31_Status_t SHT31_WaitForI2C(void)
{
    uint32_t timeout = SHT31_I2C_WAIT_TIMEOUT;

    /* Allow the Tiva I2C state machine time to latch the command. */
    SysCtlDelay(1000U);

    while (I2CMasterBusy(g_route.i2c_base) && (timeout > 0U))
    {
        timeout--;
    }

    if (timeout == 0U)
    {
        SHT31_ResetI2C2();
        return SHT31_STATUS_I2C_TIMEOUT;
    }

    if (I2CMasterErr(g_route.i2c_base) != I2C_MASTER_ERR_NONE)
    {
        return SHT31_STATUS_I2C_COMMAND_FAILED;
    }

    return SHT31_STATUS_OK;
}

static uint8_t SHT31_CalculateCRC(const uint8_t *data, uint32_t length)
{
    uint8_t crc = 0xFFU;

    for (uint32_t byte_index = 0U; byte_index < length; byte_index++)
    {
        crc ^= data[byte_index];

        for (uint8_t bit_index = 0U; bit_index < 8U; bit_index++)
        {
            if ((crc & 0x80U) != 0U)
            {
                crc = (uint8_t)((crc << 1U) ^ 0x31U);
            }
            else
            {
                crc <<= 1U;
            }
        }
    }

    return crc;
}

static SHT31_Status_t SHT31_SendCommandToAddress(uint8_t address,
                                                 uint8_t command_msb,
                                                 uint8_t command_lsb)
{
    SHT31_Status_t status;

    I2CMasterSlaveAddrSet(g_route.i2c_base, address, false);

    I2CMasterDataPut(g_route.i2c_base, command_msb);
    I2CMasterControl(g_route.i2c_base, I2C_MASTER_CMD_BURST_SEND_START);

    status = SHT31_WaitForI2C();
    if (status != SHT31_STATUS_OK)
    {
        return status;
    }

    I2CMasterDataPut(g_route.i2c_base, command_lsb);
    I2CMasterControl(g_route.i2c_base, I2C_MASTER_CMD_BURST_SEND_FINISH);

    status = SHT31_WaitForI2C();
    if (status != SHT31_STATUS_OK)
    {
        return status;
    }

    return SHT31_STATUS_OK;
}

static SHT31_Status_t SHT31_ReadBytesFromAddress(uint8_t address,
                                                 uint8_t *buffer,
                                                 uint32_t length)
{
    SHT31_Status_t status;

    if ((buffer == 0) || (length == 0U))
    {
        return SHT31_STATUS_NULL_POINTER;
    }

    I2CMasterSlaveAddrSet(g_route.i2c_base, address, true);

    if (length == 1U)
    {
        I2CMasterControl(g_route.i2c_base, I2C_MASTER_CMD_SINGLE_RECEIVE);

        status = SHT31_WaitForI2C();
        if (status != SHT31_STATUS_OK)
        {
            return (status == SHT31_STATUS_I2C_TIMEOUT) ? status : SHT31_STATUS_I2C_READ_FAILED;
        }

        buffer[0] = (uint8_t)I2CMasterDataGet(g_route.i2c_base);
        return SHT31_STATUS_OK;
    }

    I2CMasterControl(g_route.i2c_base, I2C_MASTER_CMD_BURST_RECEIVE_START);

    status = SHT31_WaitForI2C();
    if (status != SHT31_STATUS_OK)
    {
        return (status == SHT31_STATUS_I2C_TIMEOUT) ? status : SHT31_STATUS_I2C_READ_FAILED;
    }

    buffer[0] = (uint8_t)I2CMasterDataGet(g_route.i2c_base);

    for (uint32_t byte_index = 1U; byte_index < (length - 1U); byte_index++)
    {
        I2CMasterControl(g_route.i2c_base, I2C_MASTER_CMD_BURST_RECEIVE_CONT);

        status = SHT31_WaitForI2C();
        if (status != SHT31_STATUS_OK)
        {
            return (status == SHT31_STATUS_I2C_TIMEOUT) ? status : SHT31_STATUS_I2C_READ_FAILED;
        }

        buffer[byte_index] = (uint8_t)I2CMasterDataGet(g_route.i2c_base);
    }

    I2CMasterControl(g_route.i2c_base, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);

    status = SHT31_WaitForI2C();
    if (status != SHT31_STATUS_OK)
    {
        return (status == SHT31_STATUS_I2C_TIMEOUT) ? status : SHT31_STATUS_I2C_READ_FAILED;
    }

    buffer[length - 1U] = (uint8_t)I2CMasterDataGet(g_route.i2c_base);

    return SHT31_STATUS_OK;
}

static SHT31_Status_t SHT31_ProbeAddress(uint8_t address)
{
    uint8_t rx[SHT31_STATUS_READ_LENGTH];
    SHT31_Status_t status;

    status = SHT31_SendCommandToAddress(address,
                                        SHT31_COMMAND_READ_STATUS_MSB,
                                        SHT31_COMMAND_READ_STATUS_LSB);
    if (status != SHT31_STATUS_OK)
    {
        return status;
    }

    status = SHT31_ReadBytesFromAddress(address, rx, SHT31_STATUS_READ_LENGTH);
    if (status != SHT31_STATUS_OK)
    {
        return status;
    }

    if (SHT31_CalculateCRC(&rx[0], 2U) != rx[2])
    {
        return SHT31_STATUS_STATUS_CRC_FAILED;
    }

    return SHT31_STATUS_OK;
}

void SHT31_InitBus(uint32_t system_clock_hz)
{
    g_sht31_system_clock_hz = system_clock_hz;
    g_sht31_detected = false;
    g_sht31_i2c_address = 0U;

    SysCtlPeripheralEnable(g_route.gpio_periph);
    while (!SysCtlPeripheralReady(g_route.gpio_periph))
    {
    }

    SysCtlPeripheralEnable(g_route.i2c_periph);
    while (!SysCtlPeripheralReady(g_route.i2c_periph))
    {
    }

    SHT31_ResetI2C2();
    (void)SHT31_Detect();
}

const char *SHT31_GetPinRouteDescription(void)
{
    return g_route.description;
}

bool SHT31_IsDetected(void)
{
    return g_sht31_detected;
}

uint8_t SHT31_GetI2CAddress(void)
{
    return g_sht31_detected ? g_sht31_i2c_address : 0U;
}

SHT31_Status_t SHT31_Detect(void)
{
    SHT31_Status_t status;

    status = SHT31_ProbeAddress(SHT31_I2C_ADDRESS_LOW);
    if (status == SHT31_STATUS_OK)
    {
        g_sht31_i2c_address = SHT31_I2C_ADDRESS_LOW;
        g_sht31_detected = true;
        return SHT31_STATUS_OK;
    }

    status = SHT31_ProbeAddress(SHT31_I2C_ADDRESS_HIGH);
    if (status == SHT31_STATUS_OK)
    {
        g_sht31_i2c_address = SHT31_I2C_ADDRESS_HIGH;
        g_sht31_detected = true;
        return SHT31_STATUS_OK;
    }

    g_sht31_i2c_address = 0U;
    g_sht31_detected = false;
    return SHT31_STATUS_SENSOR_NOT_FOUND;
}

SHT31_Status_t SHT31_ReadBlocking(SHT31_Reading_t *reading)
{
    uint8_t rx[SHT31_READ_LENGTH];
    SHT31_Status_t status;

    if (reading == 0)
    {
        return SHT31_STATUS_NULL_POINTER;
    }

    reading->temperature_c = 0.0f;
    reading->humidity_rh = 0.0f;
    reading->raw_temperature = 0U;
    reading->raw_humidity = 0U;
    reading->valid = false;

    if (!g_sht31_detected)
    {
        status = SHT31_Detect();
        if (status != SHT31_STATUS_OK)
        {
            return status;
        }
    }

    status = SHT31_SendCommandToAddress(g_sht31_i2c_address,
                                        SHT31_COMMAND_MEAS_HIGH_MSB,
                                        SHT31_COMMAND_MEAS_HIGH_LSB);
    if (status != SHT31_STATUS_OK)
    {
        g_sht31_detected = false;
        g_sht31_i2c_address = 0U;
        return status;
    }

    SHT31_DelayMs(SHT31_MEASUREMENT_DELAY_MS);

    status = SHT31_ReadBytesFromAddress(g_sht31_i2c_address, rx, SHT31_READ_LENGTH);
    if (status != SHT31_STATUS_OK)
    {
        g_sht31_detected = false;
        g_sht31_i2c_address = 0U;
        return status;
    }

    uint8_t temp_crc = SHT31_CalculateCRC(&rx[0], 2U);
    uint8_t humidity_crc = SHT31_CalculateCRC(&rx[3], 2U);

    if (temp_crc != rx[2])
    {
        return SHT31_STATUS_TEMPERATURE_CRC_FAILED;
    }

    if (humidity_crc != rx[5])
    {
        return SHT31_STATUS_HUMIDITY_CRC_FAILED;
    }

    reading->raw_temperature = ((uint16_t)rx[0] << 8) | rx[1];
    reading->raw_humidity = ((uint16_t)rx[3] << 8) | rx[4];

    reading->temperature_c = -45.0f + (175.0f * (float)reading->raw_temperature / 65535.0f);
    reading->humidity_rh = 100.0f * (float)reading->raw_humidity / 65535.0f;
    reading->valid = true;

    return SHT31_STATUS_OK;
}

const char *SHT31_StatusToString(SHT31_Status_t status)
{
    switch (status)
    {
        case SHT31_STATUS_OK:
            return "OK";
        case SHT31_STATUS_NULL_POINTER:
            return "NULL_POINTER";
        case SHT31_STATUS_SENSOR_NOT_FOUND:
            return "SENSOR_NOT_FOUND";
        case SHT31_STATUS_I2C_TIMEOUT:
            return "I2C_TIMEOUT";
        case SHT31_STATUS_I2C_COMMAND_FAILED:
            return "I2C_COMMAND_FAILED";
        case SHT31_STATUS_I2C_READ_FAILED:
            return "I2C_READ_FAILED";
        case SHT31_STATUS_TEMPERATURE_CRC_FAILED:
            return "TEMPERATURE_CRC_FAILED";
        case SHT31_STATUS_HUMIDITY_CRC_FAILED:
            return "HUMIDITY_CRC_FAILED";
        case SHT31_STATUS_STATUS_CRC_FAILED:
            return "STATUS_CRC_FAILED";
        default:
            return "UNKNOWN";
    }
}
