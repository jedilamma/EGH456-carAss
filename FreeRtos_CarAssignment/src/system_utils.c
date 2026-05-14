
#include <stdint.h>
#include <stdarg.h>
#include "utils/uartstdio.h"
#include "system_utils.h"

void EnableHighResTimer(void)
{
    volatile uint32_t *DEMCR     = (uint32_t *)0xE000EDFC;
    volatile uint32_t *DWT_CTRL  = (uint32_t *)0xE0001000;
    volatile uint32_t *DWT_CYCCNT= (uint32_t *)0xE0001004;

    *DEMCR |= 0x01000000;
    *DWT_CYCCNT = 0;
    *DWT_CTRL |= 0x00000001;
}

void SafeUARTprintf(const char *pcString, ...)
{
    va_list vaArgP;

    if (xPrintMutex == NULL)
        return;

    if (xSemaphoreTake(xPrintMutex, pdMS_TO_TICKS(100)) == pdPASS)
    {
        va_start(vaArgP, pcString);
        UARTvprintf(pcString, vaArgP);
        va_end(vaArgP);
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