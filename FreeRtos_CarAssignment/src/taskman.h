#ifndef OPTFTOS_TASKMAN_H
#define OPTFTOS_TASKMAN_H

// Standard includes
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stdlib.h>
#include <math.h>
// Hardware includes
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "drivers/rtos_hw_drivers.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"

#include "grlib.h"
#include "widget.h"
#include "canvas.h"
#include "checkbox.h"
#include "container.h"
#include "pushbutton.h"
#include "radiobutton.h"
#include "slider.h"
#include "utils/ustdlib.h"
#include "drivers/Kentec320x240x16_ssd2119_spi.h"
#include "drivers/touch.h"
#include "drivers/opt3001.h"
#include "drivers/bmi160.h"

#include "inc/hw_nvic.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "driverlib/fpu.h"
#include "driverlib/flash.h"
#include "driverlib/systick.h"
#include "driverlib/udma.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/i2c.h"

/* Kernel includes */
#include "FreeRTOS.h"
#include "task.h"
#include "images.h"
#include "event_groups.h"
#include "semphr.h"
#include "driverlib/timer.h" 
#include "widgets.h"

// ====================== Message Object ======================
struct MsgObj
{
    uint8_t  id;
    uint32_t val;
    uint32_t avg;
    uint32_t tick;
    uint8_t  flag;
    uint32_t seq;
};

// ====================== Globals - EXTERN ONLY ======================
uint32_t g_ui32SysClock;
volatile uint32_t g_ui32TimeStamp;
volatile uint8_t  uiIndex;
volatile uint8_t  uiIndex_2;
volatile uint32_t g_pui32ButtonPressed;
// ====================== Motor Task ======================

uint16_t g_ui32Speed; 
uint32_t g_ui32Panel;
tContext sContext;


bool g_bCoolingOn; // cooling status

extern SemaphoreHandle_t xButtonSemaphore1;
extern SemaphoreHandle_t xPrintMutex;
QueueHandle_t xQueue;
extern EventGroupHandle_t xEventGroup;

// Plotting
#define PLOT_HISTORY_SIZE 30
extern uint32_t g_plotIMURawHistory[PLOT_HISTORY_SIZE];
extern uint32_t g_plotIMUFiltHistory[PLOT_HISTORY_SIZE];
extern uint32_t g_plotIMUIndex;

#define PLOT_HISTORY_SIZE 30
extern uint32_t g_plotOPTRawHistory[PLOT_HISTORY_SIZE];
extern uint32_t g_plotOPTFiltHistory[PLOT_HISTORY_SIZE];
extern uint32_t g_plotOPTIndex;

// Motor & Plot Mode
extern volatile MotorState_t g_eMotorState;
extern PlotMode_t g_eOPTPlotMode;
extern PlotMode_t g_eIMUPlotMode;
extern PlotMode_t g_ePlotMode;

// ====================== FAULT / WARNING DEFINES (MOVED HERE) ======================
#define FAULT_EBRAKE        (1U << 0)
#define FAULT_OVERCURRENT   (1U << 1)
#define FAULT_OVERTEMP      (1U << 2)
#define FAULT_SENSOR        (1U << 3)

#define WARNING_LOW_VOLTAGE (1U << 0)
#define WARNING_HIGH_TEMP   (1U << 1)

extern uint32_t g_ui32ActiveFaults;
extern uint32_t g_ui32ActiveWarnings;
extern uint32_t g_ui32ActiveSafety;

// Event Bits
#define EVENT_HIGH_THRESHOLD     (1U << 0)
#define EVENT_LOW_THRESHOLD      (1U << 1)
#define EVENT_SENSOR_MISSED      (1U << 2)
#define EVENT_QUEUE_FULL         (1U << 3)
#define EVENT_BTN_TOGGLE_PLOT    (1U << 4)
#define EVENT_DISPLAY_HOLD       (1U << 5)
#define EVENT_MOTOR_STATE_CHANGE (1U << 6)
#define EVENT_ALL_BITS           (0xFFU)

// ====================== QUEUE + TASK PRIORITIES ======================
#define mainQUEUE_LENGTH                    ( 30U )
#define mainQUEUE_SEND_TICKS_TO_WAIT        ( 0U )
#define mainQUEUE_RECEIVE_TICKS_TO_WAIT     ( 1U )
#define mainRECEIVER_DELAY_MS               ( 100U )
#define mainQUEUE_SEND_TASK_PRIORITY        ( tskIDLE_PRIORITY + 1U )
#define mainQUEUE_RECEIVE_TASK_PRIORITY     ( tskIDLE_PRIORITY + 2U )

// ====================== Function Prototypes ======================
void SafeUARTprintf(const char *pcString, ...);
void xButtonsHandler(void);
void EnableHighResTimer(void);
void prvSetMotorSpeed();

extern bool sensorOpt3001Init(void);
extern bool sensorOpt3001Read(uint16_t *rawData);
extern void sensorOpt3001Convert(uint16_t rawData, float *convertedLux);

extern bool sensorBMI160Init(void);
extern bool sensorBMI160ReadAccel(bmi160_accel_raw_t *raw);
extern void sensorBMI160ConvertAccel(const bmi160_accel_raw_t *raw, bmi160_accel_float_t *g);

extern void i2cOptDriverInit(void);
extern void calcTimeCount(void);
extern void SetTimeFromCompileTime(void);
void xTimerHandler(void);
extern void UpdateTime(void);

extern tCanvasWidget g_sTimeCanvas;
extern tCanvasWidget g_sMotorStateCanvas;

#endif // OPTFTOS_TASKMAN_H