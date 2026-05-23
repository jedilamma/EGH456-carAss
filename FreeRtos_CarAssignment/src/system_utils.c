
#include <stdint.h>
#include <stdarg.h>
#include "utils/uartstdio.h"
#include "system_utils.h"

// Moving average state for OPT3001 - Originally average divided by 10 even before 10 samples existed, 
// leading to inaccurate initial readings
#define AVG_WINDOW 10
static uint32_t LuxBuffer[AVG_WINDOW] = {0};
static uint32_t RangeBuffer[AVG_WINDOW] = {0};
static uint32_t IMUBuffer[AVG_WINDOW] = {0};

static uint32_t RSum = 0;
static uint8_t RIndex = 0;
static uint8_t RCount = 0;
static uint32_t LSum = 0;
static uint8_t LIndex = 0;
static uint8_t LCount = 0;
static uint32_t BSum = 0;
static uint8_t BIndex = 0;
static uint8_t BCount = 0;



void EnableHighResTimer(void)
{
    volatile uint32_t *DEMCR     = (uint32_t *)0xE000EDFC;
    volatile uint32_t *DWT_CTRL  = (uint32_t *)0xE0001000;
    volatile uint32_t *DWT_CYCCNT= (uint32_t *)0xE0001004;

    *DEMCR |= 0x01000000;
    *DWT_CYCCNT = 0;
    *DWT_CTRL |= 0x00000001;
}

// ====================== SHARED PRINT BUFFER ======================
char g_pcPrintBuffer[PRINT_BUFFER_SIZE];

// Improved SafeUARTprintf using shared buffer + vsnprintf
void SafeUARTprintf(const char *pcString, ...)
{
    va_list vaArgP;

    if (xPrintMutex == NULL)
    {
        // Fallback if mutex not ready yet
        UARTprintf("Print Mutex not ready!\r\n");
        return;
    }

    // Take mutex - this protects the shared buffer
    if (xSemaphoreTake(xPrintMutex, portMAX_DELAY) == pdPASS)
    {
        // Format into the shared buffer (much lighter than UARTvprintf)
        va_start(vaArgP, pcString);
        uvsnprintf(g_pcPrintBuffer, PRINT_BUFFER_SIZE, pcString, vaArgP);
        va_end(vaArgP);

        // Now send the already-formatted string to UART
        // This still uses some stack but far less than before
        UARTprintf("%s", g_pcPrintBuffer);

        xSemaphoreGive(xPrintMutex);
    }
}




//=============================================================
// Set initial time from compiler macros (__DATE__ and __TIME__)
//=============================================================
void SetTimeFromCompileTime(void)
{
    const char* date = __DATE__;
    const char* time = __TIME__;

    g_ui8Hours   = (time[0]-'0')*10 + (time[1]-'0');
    g_ui8Minutes = (time[3]-'0')*10 + (time[4]-'0');
    g_ui8Seconds = (time[6]-'0')*10 + (time[7]-'0');

    g_ui8Day = (date[4]-'0')*10 + (date[5]-'0');

    g_ui16Year = (date[7]-'0')*1000 + (date[8]-'0')*100 + 
                 (date[9]-'0')*10   + (date[10]-'0');

    /* Month parser */
    if      (date[0]=='J' && date[1]=='a') g_ui8Month = 1;
    else if (date[0]=='F')                 g_ui8Month = 2;
    else if (date[0]=='M' && date[2]=='r') g_ui8Month = 3;
    else if (date[0]=='A' && date[1]=='p') g_ui8Month = 4;
    else if (date[0]=='M' && date[2]=='y') g_ui8Month = 5;
    else if (date[0]=='J' && date[2]=='n') g_ui8Month = 6;
    else if (date[0]=='J' && date[2]=='l') g_ui8Month = 7;
    else if (date[0]=='A' && date[1]=='u') g_ui8Month = 8;
    else if (date[0]=='S')                 g_ui8Month = 9;
    else if (date[0]=='O')                 g_ui8Month = 10;
    else if (date[0]=='N')                 g_ui8Month = 11;
    else if (date[0]=='D')                 g_ui8Month = 12;
    else                                   g_ui8Month = 1;  /* default */
}

void calcTimeCount(void)
{
    static uint8_t tickCounter = 0;
    tickCounter++;

    if (tickCounter >= 5)  
    {
        tickCounter = 0;

        g_ui8Seconds++;
        if (g_ui8Seconds >= 60)
        {
            g_ui8Seconds = 0;
            g_ui8Minutes++;
            if (g_ui8Minutes >= 60)
            {
                g_ui8Minutes = 0;
                g_ui8Hours++;
                if (g_ui8Hours >= 24)
                {
                    g_ui8Hours = 0;
                    g_ui8Day++;

                    uint8_t daysInMonth[12] = {31,28,31,30,31,30,31,31,30,31,30,31};
                    if (g_ui8Day > daysInMonth[g_ui8Month-1])
                    {
                        g_ui8Day = 1;
                        g_ui8Month++;
                        if (g_ui8Month > 12)
                        {
                            g_ui8Month = 1;
                            g_ui16Year++;
                        }
                    }
                }
            }
        }
    }
}

// Fast integer square root (for magnitude calculation)
uint32_t isqrt(uint32_t n)
{
    uint32_t res = 0;
    uint32_t bit = 1u << 30;        // start at highest power of 4 <= 2^31

    while (bit > n) bit >>= 2;

    while (bit != 0)
    {
        if (n >= res + bit)
        {
            n -= res + bit;
            res = (res >> 1) + bit;
        }
        else
        {
            res >>= 1;
        }
        bit >>= 2;
    }
    return res;
}


int32_t FloatToX100(float value)
{
    // Convert signed float to integer x100
    if (value >= 0.0f)
    {
        return (int32_t)((value * 100.0f) + 0.5f);
    }

    return (int32_t)((value * 100.0f) - 0.5f);
}

uint32_t FloatToUX100(float value)
{
    // Convert unsigned float to integer x100
    if (value <= 0.0f)
    {
        return 0U;
    }

    return (uint32_t)((value * 100.0f) + 0.5f);
}

// ====================== OPT3001 Task ======================
uint32_t UpdateAverage(uint32_t Value,uint8_t SensorState)
{
    // Only divide by the number of valid samples during startup
    switch(SensorState)
    {
        case SENSOR_ID_OPT3001:
            if (LCount < AVG_WINDOW)
            {
                LuxBuffer[LIndex] = Value;
                LSum += Value;
                LCount++;
            }
            else
            {
                LSum -= LuxBuffer[LIndex];
                LuxBuffer[LIndex] = Value;
                LSum += Value;
            }

            LIndex++;
            if (LIndex >= AVG_WINDOW)
            {
                LIndex = 0;
            }
            return LSum / LCount;

            break;
        case SENSOR_ID_BMI160:
            if (BCount < AVG_WINDOW)
            {
                IMUBuffer[BIndex] = Value;
                BSum += Value;
                BCount++;
            }
            else
            {
                BSum -= IMUBuffer[BIndex];
                LuxBuffer[BIndex] = Value;
                BSum += Value;
            }

            BIndex++;
            if (BIndex >= AVG_WINDOW)
            {
                BIndex = 0;
            }
            return BSum / BCount;
        break;
        case SENSOR_ID_RANGE:
            if (RCount < AVG_WINDOW)
            {
                RangeBuffer[RIndex] = Value;
                RSum += Value;
                RCount++;
            }
            else
            {
                RSum -= RangeBuffer[RIndex];
                RangeBuffer[RIndex] = Value;
                RSum += Value;
            }

            RIndex++;
            if (RIndex >= AVG_WINDOW)
            {
                RIndex = 0;
            }
            return RSum / RCount;
        break;
    }
    return 0;
}

