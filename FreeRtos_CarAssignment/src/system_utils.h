#ifndef UTILS_H
#define UTILS_H

#include "taskman.h"
#include "stdarg.h"

// ====================== Time Variables (EXTERN) ======================
volatile uint8_t  g_ui8Hours;
volatile uint8_t  g_ui8Minutes;
volatile uint8_t  g_ui8Seconds;
volatile uint8_t  g_ui8Day;
volatile uint8_t  g_ui8Month;
volatile uint16_t g_ui16Year;
volatile int16_t  g_i16MotorValue;

// ====================== Function Prototypes ======================
void EnableHighResTimer(void);
void SafeUARTprintf(const char *pcString, ...);
void SetTimeFromCompileTime(void);
void calcTimeCount(void);
uint32_t isqrt(uint32_t n);

#endif // UTILS_H