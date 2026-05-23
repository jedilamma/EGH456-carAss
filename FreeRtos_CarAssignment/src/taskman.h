#ifndef OPTFTOS_TASKMAN_H
#define OPTFTOS_TASKMAN_H

// Define sensor IDs to reduce dependency on magic numbers
#define SENSOR_ID_OPT3001 1U // Optical Sensor (OPT3001)
#define SENSOR_ID_BMI160 2U // Accelerometer/Magnetometer (BMI160)
#define SENSOR_ID_RANGE 3U // Range Sensor
#define SENSOR_ID_HALL 4U // MotorSensor
#define SENSOR_ID_SHT31 5U // Temperature/Humidity Sensor (SHT31)

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
#include "drivers/i2cOptDriver.h"

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
#include "system_utils.h"
#include "drivers/temp_humidity.h" 

// ====================== Message Object ======================
typedef struct __attribute__((aligned(4))) MsgObj
{
    uint8_t  id;        // Sensor ID (OPT3001, IMU, SHT31, etc.)
    uint8_t  flag;      // 0 = OK, 1 = Error

    int32_t val;       // Main value (e.g. raw lux, X accel, raw temp)
    int32_t avg;       // Filtered / Average value

    int32_t val2;      // Secondary value (e.g. Z accel, raw humidity)
    int32_t avg2;      // Secondary filtered value

    uint32_t tick;      // Optional: timestamp or sample counter
    uint32_t seq;       // Optional: sequence number
    
} MsgObj;

// ====================== Globals - EXTERN ONLY ======================
extern uint32_t g_ui32SysClock;
extern volatile uint32_t g_ui32TimeStamp;
extern volatile uint8_t  uiIndex;
extern volatile uint8_t  uiIndex_2;
extern volatile uint32_t g_pui32ButtonPressed;
// ====================== Motor Task ======================

extern uint16_t g_ui32Speed;
extern uint32_t g_ui32Panel;
extern tContext sContext;

// GUI-facing SHT31 state variables
extern bool g_bCoolingOn;
extern bool g_bHeatingOn;
extern bool g_bSHT31Valid;
extern bool g_faultSwitch;
extern bool g_startDisplay;
extern bool g_bFaultAcknowledged;

extern int32_t  g_i32CabinTempCx100;
extern uint32_t g_ui32CabinHumidityRHx100;

extern int32_t  g_i32HeatingThresholdCx100;
extern int32_t  g_i32CoolingThresholdCx100;

// Dedicated event bit for signaling new sensor data is available
#define EVENT_SENSOR_DATA_READY (1U << 7)

extern QueueHandle_t xQueue;

extern SemaphoreHandle_t xI2CMutex;
extern SemaphoreHandle_t xButtonSemaphore1;
extern SemaphoreHandle_t xPrintMutex;
extern EventGroupHandle_t xEventGroup;

// Plotting
#define PLOT_HISTORY_SIZE 30
extern uint32_t g_plotIMUxHistory[PLOT_HISTORY_SIZE];
extern uint32_t g_plotIMUyHistory[PLOT_HISTORY_SIZE];
extern uint32_t g_plotIMUzHistory[PLOT_HISTORY_SIZE];
extern uint32_t g_plotIMUmagHistory[PLOT_HISTORY_SIZE];
extern uint32_t g_plotIMUIndex;

extern uint32_t g_plotOPTRawHistory[PLOT_HISTORY_SIZE];
extern uint32_t g_plotOPTFiltHistory[PLOT_HISTORY_SIZE];
extern uint32_t g_plotOPTIndex;

extern uint32_t g_plotRNGRawHistory[PLOT_HISTORY_SIZE];
extern uint32_t g_plotRNGFiltHistory[PLOT_HISTORY_SIZE];
extern uint32_t g_plotRNGIndex;

// Motor & Plot Mode
extern volatile MotorState_t g_eMotorState;
extern PlotMode_t g_eOPTPlotMode;
extern PlotMode_t g_eIMUPlotMode;
extern PlotMode_t g_eRNGPlotMode;
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
#define EVENT_LUXHIGH_THRESHOLD     (1U << 0)
#define EVENT_LUXLOW_THRESHOLD      (1U << 1)
#define EVENT_SENSOR_MISSED      (1U << 2)
#define EVENT_QUEUE_FULL         (1U << 3)
#define EVENT_BTN_TOGGLE_PLOT    (1U << 4)
#define EVENT_DISPLAY_HOLD       (1U << 5)
#define EVENT_MOTOR_STATE_CHANGE (1U << 6)
#define EVENT_HIGH_ACCEL_LIMIT (1U << 7)
#define EVENT_ALL_BITS           (0xFFU)

// ====================== QUEUE + TASK PRIORITIES ======================
#define mainQUEUE_LENGTH                    ( 30U )
#define mainQUEUE_SEND_TICKS_TO_WAIT        ( 0U )
#define mainQUEUE_RECEIVE_TICKS_TO_WAIT     ( 1U )
#define mainRECEIVER_DELAY_MS               ( 10U )
#define mainQUEUE_SEND_TASK_PRIORITY        ( tskIDLE_PRIORITY + 1U )
#define mainQUEUE_RECEIVE_TASK_PRIORITY     ( tskIDLE_PRIORITY + 2U )

// ====================== Function Prototypes ======================
void SafeUARTprintf(const char *pcString, ...);
void xButtonsHandler(void);
void EnableHighResTimer(void);
//void prvSetMotorSpeed();
void xBMI160DRDYHandler(void);

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