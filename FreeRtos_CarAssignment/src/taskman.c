#include "taskman.h"
#include "temp_humidity.h" // Adds the SHT31 temperature/humidity wrapper
#include "motorlib.h"      // Motor control library header

// Motor Control Constants
#define MOTOR_TASK_PERIOD_MS              5U
#define MOTOR_SPEED_UPDATE_PERIOD_MS      10U

#define MOTOR_PWM_PERIOD_US               50U
#define MOTOR_MIN_DUTY_US                 5U
#define MOTOR_START_DUTY_US               7U

#define MOTOR_MAX_RPM                     3000U
#define MOTOR_HALL_EDGES_PER_REV          6U

#define MOTOR_ACCEL_LIMIT_RPM_PER_S       500U
#define MOTOR_DECEL_LIMIT_RPM_PER_S       500U
#define MOTOR_ESTOP_DECEL_RPM_PER_S       1000U

#define MOTOR_START_FEEDBACK_EDGES        3U
#define MOTOR_STOPPED_RPM_THRESHOLD       30U

#define MOTOR_PLOT_PERIOD_MS              50U

// Pin definitions
#define HALL_A_GPIO_PERIPH     SYSCTL_PERIPH_GPIOM
#define HALL_A_GPIO_BASE       GPIO_PORTM_BASE
#define HALL_A_GPIO_PIN        GPIO_PIN_3

#define HALL_BC_GPIO_PERIPH    SYSCTL_PERIPH_GPIOH
#define HALL_BC_GPIO_BASE      GPIO_PORTH_BASE
#define HALL_B_GPIO_PIN        GPIO_PIN_2
#define HALL_C_GPIO_PIN        GPIO_PIN_3

// Hardware drivers for buttons and LEDs
static void prvOptTask(void *pvParameters);
static void prvLEDTasksw1(void *pvParameters);
static void readerTask(void *pvParameters);
static void prvDisplayTask(void *pvParameters);
static void prvMotorTask(void *pvParameters);
// static void prvFaultSimulatorTask(void *pvParameters);
static void prvIMUTask(void *pvParameters);
static void prvSHT31Task(void *pvParameters); // SHT31 task prototype
static void prvConfigureHallSensors(void); // Hall sensor configuration prototype

// Data format helpers for OPT3001 and SHT31
static uint32_t UpdateLuxAverage(uint32_t newLux);

static void prvConfigureLED(void);
static void prvConfigureButton(void);
static void prvConfigureOPT(void);
static void prvConfigure(void);
static void prvConfigureHWTimer(void);

EventGroupHandle_t xEventGroup = NULL;
uint32_t g_ui32Panel = 0;
uint32_t g_ui32ActiveFaults = 0;
uint32_t g_ui32ActiveWarnings = 0;
uint32_t g_ui32ActiveSafety = 0;
// ====================== NEW ENGINE / SAFETY / LUX GLOBALS ======================
uint32_t g_ui32FilteredBodyAccel     = 420;   // live filtered acceleration
uint32_t g_ui32CrashThreshold        = 2500;   // configurable crash limit
uint32_t g_ui32CurrentDistance       = 180;   // live distance to object (cm)
uint32_t g_ui32DetRangeVal           = 90;    // configurable safe distance
uint32_t g_ui32LuxMinThreshold       = 10;    // low lux trigger
uint32_t g_ui32LuxMaxThreshold       = 950;   // high lux trigger
uint32_t g_ui32EngineTemp            = 55;    // simulated engine temp for heating/cooling
uint32_t g_ui32HeatingThreshold      = 100;
uint32_t g_ui32CoolingThreshold      = 10;
uint32_t g_ui32AccelMaxVal           = 10;

// Already declared in taskman.h as extern
volatile uint32_t g_ui32TimeStamp = 0;
volatile uint8_t  uiIndex = 0;
volatile uint8_t  uiIndex_2 = 0;
volatile uint32_t g_pui32ButtonPressed = 0;

uint32_t g_plotOPTRawHistory[PLOT_HISTORY_SIZE] = {0};
uint32_t g_plotOPTFiltHistory[PLOT_HISTORY_SIZE] = {0};
uint32_t g_plotOPTIndex = 0;

uint32_t g_plotIMURawHistory[PLOT_HISTORY_SIZE] = {0};
uint32_t g_plotIMUFiltHistory[PLOT_HISTORY_SIZE] = {0};
uint32_t g_plotIMUIndex = 0;

uint32_t g_plotIMUxHistory[PLOT_HISTORY_SIZE] = {0};
uint32_t g_plotIMUyHistory[PLOT_HISTORY_SIZE] = {0};

PlotMode_t g_eOPTPlotMode = OPT_PLOT_MODE_FILTERED_ONLY;
PlotMode_t g_eIMUPlotMode = IMU_PLOT_MODE_FILTERED_ONLY; // Fixed typo in declaration as taskman.h expects g_eIMUPlotMode
PlotMode_t g_ePlotMode = IMU_PLOT_MODE_FILTERED_ONLY;
volatile MotorState_t g_eMotorState = MOTOR_IDLE;

// Moving average state for OPT3001 - Originally average divided by 10 even before 10 samples existed, 
// leading to inaccurate initial readings
#define OPT_AVG_WINDOW 10
static uint32_t luxBuffer[OPT_AVG_WINDOW] = {0};
static uint32_t luxSum = 0;
static uint8_t luxIndex = 0;
static uint8_t luxCount = 0;

bool g_bOPTDisplayHold = false;
bool g_bIMUDisplayHold = false;
bool g_bDisplayHold = false;

// GUI-facing SHT31 state variables
bool g_bHeatingOn = false;
bool g_bCoolingOn = false;
bool g_bSHT31Valid = false;

// Store temp/humidity as integer x100 to avoid floating point in display code
int32_t  g_i32CabinTempCx100 = 0;
uint32_t g_ui32CabinHumidityRHx100 = 0;

int32_t  g_i32HeatingThresholdCx100 = 1800;
int32_t  g_i32CoolingThresholdCx100 = 2600;

uint16_t g_ui32Speed = 0; // Gui-facing speed variable (RPM)

// Motor Control Variables
extern volatile uint32_t g_ui32DesiredSpeedRPM; // User request from slider
extern volatile bool g_bMotorStartRequested;
extern volatile bool g_bMotorStopRequested;
extern volatile bool g_bMotorFaultAckRequested;
volatile bool g_bFaultAcknowledged = false;

extern volatile uint32_t g_ui32MotorReferenceRPM; // Ramped reference speed
extern volatile uint32_t g_ui32MotorActualRPM; // Measured hall sensor speed
extern volatile uint32_t g_ui32MotorPWMPercent;
extern volatile uint32_t g_ui32MotorPowerW;

// Motor Runtime Globals
volatile uint32_t g_ui32DesiredSpeedRPM = 0;
volatile bool g_bMotorStartRequested = false;
volatile bool g_bMotorStopRequested = false;
volatile bool g_bMotorFaultAckRequested = false;

volatile uint32_t g_ui32MotorReferenceRPM = 0;
volatile uint32_t g_ui32MotorActualRPM = 0;
volatile uint32_t g_ui32MotorPWMPercent = 0;
volatile uint32_t g_ui32MotorPowerW = 0;

static volatile uint32_t g_ui32HallEdgeCount = 0;
static volatile uint32_t g_ui32HallEdgeTotal = 0;
static bool g_bMotorLibReady = false;

static int32_t g_i32ReferenceRPMx100 = 0;
static int32_t g_i32PIIntegral = 0;
static uint16_t g_ui16MotorDutyUs = 0;

tContext sContext;

QueueHandle_t xQueue = NULL;

static void prvConfigureHallSensors(void)
{
    SysCtlPeripheralEnable(HALL_A_GPIO_PERIPH);
    SysCtlPeripheralEnable(HALL_BC_GPIO_PERIPH);

    while (!SysCtlPeripheralReady(HALL_A_GPIO_PERIPH)) {}
    while (!SysCtlPeripheralReady(HALL_BC_GPIO_PERIPH)) {}

    GPIOPinTypeGPIOInput(HALL_A_GPIO_BASE, HALL_A_GPIO_PIN);
    GPIOPinTypeGPIOInput(HALL_BC_GPIO_BASE, HALL_B_GPIO_PIN | HALL_C_GPIO_PIN);

    GPIOPadConfigSet(HALL_A_GPIO_BASE,
                     HALL_A_GPIO_PIN,
                     GPIO_STRENGTH_2MA,
                     GPIO_PIN_TYPE_STD_WPU);

    GPIOPadConfigSet(HALL_BC_GPIO_BASE,
                     HALL_B_GPIO_PIN | HALL_C_GPIO_PIN,
                     GPIO_STRENGTH_2MA,
                     GPIO_PIN_TYPE_STD_WPU);

    GPIOIntDisable(HALL_A_GPIO_BASE, HALL_A_GPIO_PIN);
    GPIOIntDisable(HALL_BC_GPIO_BASE, HALL_B_GPIO_PIN | HALL_C_GPIO_PIN);

    GPIOIntClear(HALL_A_GPIO_BASE, HALL_A_GPIO_PIN);
    GPIOIntClear(HALL_BC_GPIO_BASE, HALL_B_GPIO_PIN | HALL_C_GPIO_PIN);

    GPIOIntTypeSet(HALL_A_GPIO_BASE, HALL_A_GPIO_PIN, GPIO_BOTH_EDGES);
    GPIOIntTypeSet(HALL_BC_GPIO_BASE,
                   HALL_B_GPIO_PIN | HALL_C_GPIO_PIN,
                   GPIO_BOTH_EDGES);

    GPIOIntEnable(HALL_A_GPIO_BASE, HALL_A_GPIO_PIN);
    GPIOIntEnable(HALL_BC_GPIO_BASE, HALL_B_GPIO_PIN | HALL_C_GPIO_PIN);

    IntEnable(INT_GPIOM);
    IntEnable(INT_GPIOH);
}

void vCreateTask(void)
{
    prvConfigureLED();
    prvConfigureButton();
    prvConfigureOPT();
    prvConfigure();   
    prvConfigureHWTimer();         
    prvConfigureHallSensors();         
    SetTimeFromCompileTime();
    
    xTaskCreate(prvLEDTasksw1,      "LED",     configMINIMAL_STACK_SIZE * 2, NULL, tskIDLE_PRIORITY + 2, NULL);
    xTaskCreate(prvIMUTask,         "IMU",     configMINIMAL_STACK_SIZE * 6, NULL, tskIDLE_PRIORITY + 2, NULL); 
    xTaskCreate(prvOptTask,         "OPT",     configMINIMAL_STACK_SIZE * 6, NULL, tskIDLE_PRIORITY + 2, NULL);    
    xTaskCreate(prvDisplayTask,     "DISPLAY", configMINIMAL_STACK_SIZE * 12, NULL, tskIDLE_PRIORITY + 3, NULL);  // biggest
 // xTaskCreate(prvFaultSimulatorTask, "FAULTSIM", configMINIMAL_STACK_SIZE * 4, NULL, tskIDLE_PRIORITY + 2, NULL);
    xTaskCreate(prvMotorTask, "MOTOR", configMINIMAL_STACK_SIZE * 6, NULL, tskIDLE_PRIORITY + 4, NULL);
    xTaskCreate(prvSHT31Task,       "SHT31",    configMINIMAL_STACK_SIZE * 4, NULL, tskIDLE_PRIORITY + 2, NULL); // Create the SHT31 sampling/filtering task

    SafeUARTprintf("All tasks created successfully!\r\n");
    SafeUARTprintf("Free Heap AFTER tasks: %u bytes\r\n", xPortGetFreeHeapSize());
}

void prvLEDTasksw1(void *pvParameters)
{
    for (;;)
    {
        if (xSemaphoreTake(xButtonSemaphore1, portMAX_DELAY) == pdPASS)
        {
            if (g_pui32ButtonPressed == USR_SW1)
            {
                uiIndex = !uiIndex;
                SafeUARTprintf(uiIndex ? " ---- Paused Sensor ----\r\n" : " ---- Reading Sensor ----\r\n");
            }
            if (g_pui32ButtonPressed == USR_SW2)
            {
                uiIndex_2 = !uiIndex_2;
                SafeUARTprintf(uiIndex_2 ? "Sensor output 2 - Reading Thresholds\r\n" : "Sensor output 1 - Serial plot\r\n");
            }

            LEDWrite(LED_D1|LED_D2|LED_D3|LED_D4, 0);

            if (!uiIndex)  LEDWrite(LED_D1, LED_D1);
            if (!uiIndex_2) LEDWrite(LED_D2, LED_D2);
        }
    }
}

// New FreeRTOS task for the SHT31 temperature/humidity sensor
static void prvSHT31Task(void *pvParameters)
{
    (void)pvParameters;

    TempHumiditySample_t sample;
    struct MsgObj xMsgObj = {0};
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint32_t sampleCount = 0;

    SafeUARTprintf("=== SHT31 Task Started ===\r\n");
    SafeUARTprintf("SHT31 route: %s\r\n", SHT31_GetPinRouteDescription());

    // Initialise the SHT31 driver using the system clock
    TempHumidity_Init(g_ui32SysClock);

    if (SHT31_IsDetected())
    {
        SafeUARTprintf("SHT31 detected at I2C address 0x%02x\r\n",
                       (unsigned int)SHT31_GetI2CAddress());
    }
    else
    {
        SafeUARTprintf("SHT31 not detected during init\r\n");
    }

    for (;;)
    {
        // Update the SHT31 measurement and filtering state
        TempHumidity_Update();
        TempHumidity_GetLatest(&sample);

        sampleCount++;

        // Send SHT31 through the same queue as OPT3001 and IMU
        xMsgObj.id   = SENSOR_ID_SHT31;
        xMsgObj.tick = sampleCount;
        xMsgObj.seq  = (uint32_t)sample.status;

        if (sample.valid)
        {
            // Convert floats to x100 integers before queueing
            int32_t rawTemp_x100  = FloatToX100(sample.raw_temperature_c);
            int32_t filtTemp_x100 = FloatToX100(sample.filtered_temperature_c);

            uint32_t rawHum_x100  = FloatToUX100(sample.raw_humidity_rh);
            uint32_t filtHum_x100 = FloatToUX100(sample.filtered_humidity_rh);

            xMsgObj.val  = rawTemp_x100;          // raw temperature x100
            xMsgObj.avg  = (uint32_t)filtTemp_x100; // filtered temperature x100
            xMsgObj.val2 = (int32_t)rawHum_x100;  // raw humidity x100
            xMsgObj.avg2 = (int32_t)filtHum_x100; // filtered humidity x100
            xMsgObj.flag = 0U;                    // valid reading
        }
        else
        {
            // Send a flagged fault message instead of silently failing
            xMsgObj.val  = 0;
            xMsgObj.avg  = 0;
            xMsgObj.val2 = 0;
            xMsgObj.avg2 = 0;
            xMsgObj.flag = 1U;
        }

        if (xQueueSend(xQueue, &xMsgObj, 0) != pdPASS)
        {
            // Report queue full through the event group
            xEventGroupSetBits(xEventGroup, EVENT_QUEUE_FULL);
        }
        else
        {
            // Wake readerTask when new sensor data exists
            xEventGroupSetBits(xEventGroup, EVENT_SENSOR_DATA_READY);
        }

        // SHT31 updates at 1 Hz which is enough for GUI display
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000));
    }
}

void xButtonsHandler(void)
{
    // (Your existing ISR code - unchanged)
    BaseType_t xLEDTaskWoken = pdFALSE;
    uint32_t ui32Status = GPIOIntStatus(BUTTONS_GPIO_BASE, true);
    GPIOIntClear(BUTTONS_GPIO_BASE, ui32Status);

    if ((xTaskGetTickCount() - g_ui32TimeStamp) > 100)
    {
        if (ui32Status & USR_SW1) g_pui32ButtonPressed = USR_SW1;
        else if (ui32Status & USR_SW2) g_pui32ButtonPressed = USR_SW2;

        xSemaphoreGiveFromISR(xButtonSemaphore1, &xLEDTaskWoken);
        portYIELD_FROM_ISR(xLEDTaskWoken);
    }
    g_ui32TimeStamp = xTaskGetTickCount();
}
// readerTask now drains the queue and keeps the newest message per sensor.
// IMU runs faster than OPT3001 and SHT31, so a "single latest message" reader
// can accidentally discard SHT31 before the GUI ever sees it.
static void readerTask(void *pvParameters)
{
    struct MsgObj xCurrentMsgObj;
    struct MsgObj latestSensorData[SENSOR_ID_SHT31 + 1];
    bool hasLatest[SENSOR_ID_SHT31 + 1] = {false};

    BaseType_t xRecvStatus;
    EventBits_t uxBits;
    uint32_t messagesDiscarded = 0;
    uint16_t missingSampleCount = 0;

    (void)pvParameters;

    SafeUARTprintf("Reader Task Started - Draining queue, keeping latest per sensor...\r\n");

    for (;;)
    {
        // Wait for sensor/event updates, but timeout regularly
        uxBits = xEventGroupWaitBits(xEventGroup,
                                     (EVENT_SENSOR_MISSED |
                                      EVENT_QUEUE_FULL |
                                      EVENT_HIGH_THRESHOLD |
                                      EVENT_LOW_THRESHOLD |
                                      EVENT_SENSOR_DATA_READY |
                                      EVENT_DISPLAY_HOLD |
                                      EVENT_MOTOR_STATE_CHANGE),
                                     pdTRUE,
                                     pdFALSE,
                                     pdMS_TO_TICKS(100));

        if (uxBits & EVENT_MOTOR_STATE_CHANGE)
        {
            if (g_eMotorState == MOTOR_ESTOP_BRAKING || g_eMotorState == MOTOR_FAULT_LATCHED)
            {
                g_ui32ActiveFaults |= FAULT_EBRAKE;
            }
            else
            {
                g_ui32ActiveFaults &= ~FAULT_EBRAKE;
            }
        }

        if (uxBits & EVENT_QUEUE_FULL)
        {
            SafeUARTprintf("EVENT: QUEUE FULL! Message dropped.\r\n");
        }

        if (uxBits & EVENT_HIGH_THRESHOLD)
        {
            SafeUARTprintf("EVENT: HIGH THRESHOLD crossed! (Filtered > 1000 lux)\r\n");
        }

        if (uxBits & EVENT_LOW_THRESHOLD)
        {
            SafeUARTprintf("EVENT: LOW THRESHOLD crossed! (Filtered < 10 lux)\r\n");
        }

        // Reset the local "latest message" cache each reader cycle
        for (uint8_t id = SENSOR_ID_OPT3001; id <= SENSOR_ID_SHT31; id++)
        {
            hasLatest[id] = false;
        }

        messagesDiscarded = 0;

        // Drain all pending queue messages
        xRecvStatus = xQueueReceive(xQueue, &xCurrentMsgObj, 0);

        if (xRecvStatus != pdPASS)
        {
            missingSampleCount++;
        }

        while (xRecvStatus == pdPASS)
        {
            if ((xCurrentMsgObj.id >= SENSOR_ID_OPT3001) &&
                (xCurrentMsgObj.id <= SENSOR_ID_SHT31))
            {
                if (hasLatest[xCurrentMsgObj.id])
                {
                    // Count only replacements for the same sensor ID as discarded
                    messagesDiscarded++;
                }

                latestSensorData[xCurrentMsgObj.id] = xCurrentMsgObj;
                hasLatest[xCurrentMsgObj.id] = true;
            }

            xRecvStatus = xQueueReceive(xQueue, &xCurrentMsgObj, 0);
        }

        // Process the newest message for each sensor independently
        for (uint8_t id = SENSOR_ID_OPT3001; id <= SENSOR_ID_SHT31; id++)
        {
            if (!hasLatest[id])
            {
                continue;
            }

            struct MsgObj *pData = &latestSensorData[id];

            switch (id)
            {
                case SENSOR_ID_OPT3001:
                {
                    if (pData->flag == 1U)
                    {
                        missingSampleCount++;
                        break;
                    }

                    g_plotOPTRawHistory[g_plotOPTIndex] = (uint32_t)pData->val;
                    g_plotOPTFiltHistory[g_plotOPTIndex] = pData->avg;
                    g_plotOPTIndex = (g_plotOPTIndex + 1) % PLOT_HISTORY_SIZE;

                    if (g_ui32Panel == 2)
                    {
                        WidgetPaint((tWidget *)&g_sCanvas3);
                    }

                    break;
                }

                case SENSOR_ID_IMU:
                {
                    if (pData->flag == 1U)
                    {
                        break;
                    }

                    uint32_t imuX = (uint32_t)pData->val;
                    uint32_t imuY = (uint32_t)pData->avg;
                    uint32_t imuZ = (uint32_t)pData->val2;
                    uint32_t imuMag = (uint32_t)pData->avg2;

                    g_plotIMUxHistory[g_plotIMUIndex] = imuX;
                    g_plotIMUyHistory[g_plotIMUIndex] = imuY;

                    g_plotIMURawHistory[g_plotIMUIndex] = imuZ;
                    g_plotIMUFiltHistory[g_plotIMUIndex] = imuMag;

                    g_ui32FilteredBodyAccel = imuMag;

                    g_plotIMUIndex = (g_plotIMUIndex + 1) % PLOT_HISTORY_SIZE;

                    if (g_ui32Panel == 3)
                    {
                        WidgetPaint((tWidget *)&g_sCanvas3);
                    }

                    break;
                }

                case SENSOR_ID_SHT31:
                {
                    // Extract SHT31 filtered values from the message
                    int32_t temp_x100 = (int32_t)pData->avg;
                    uint32_t hum_x100 = (uint32_t)pData->avg2;

                    // Update GUI-facing globals
                    g_i32CabinTempCx100 = temp_x100;
                    g_ui32CabinHumidityRHx100 = hum_x100;
                    g_bSHT31Valid = (pData->flag == 0U);

                    if (g_bSHT31Valid)
                    {
                        g_ui32ActiveFaults &= ~FAULT_SENSOR;

                        // Climate status logic for GUI display
                        if (g_i32CabinTempCx100 >= g_i32CoolingThresholdCx100)
                        {
                            g_bCoolingOn = true;
                            g_bHeatingOn = false;
                            g_ui32ActiveWarnings |= WARNING_HIGH_TEMP;
                        }
                        else if (g_i32CabinTempCx100 <= g_i32HeatingThresholdCx100)
                        {
                            g_bHeatingOn = true;
                            g_bCoolingOn = false;
                            g_ui32ActiveWarnings &= ~WARNING_HIGH_TEMP;
                        }
                        else
                        {
                            g_bHeatingOn = false;
                            g_bCoolingOn = false;
                            g_ui32ActiveWarnings &= ~WARNING_HIGH_TEMP;
                        }
                    }
                    else
                    {
                        // Failed SHT31 read becomes a sensor fault
                        g_ui32ActiveFaults |= FAULT_SENSOR;
                        g_bCoolingOn = false;
                        g_bHeatingOn = false;
                    }

                    // Repaint only the active panel
                    if (g_ui32Panel == 0)
                    {
                        WidgetPaint((tWidget *)&g_sDashboard);
                    }

                    if (g_ui32Panel == 1)
                    {
                        WidgetPaint((tWidget *)&g_sEngineCanvas);
                    }

                    break;
                }

                default:
                {
                    break;
                }
            }
        }
        if ((g_ui32FilteredBodyAccel > g_ui32CrashThreshold) ||
            (g_ui32CurrentDistance < g_ui32DetRangeVal) ||
            (g_ui32MotorPowerW > 0U && g_ui32MotorPowerW > 120U))   // temporary threshold until GUI power slider exists
        {
            g_ui32ActiveFaults |= FAULT_EBRAKE;

            if (xEventGroup)
            {
                xEventGroupSetBits(xEventGroup, EVENT_FAULT_ESTOP);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(mainRECEIVER_DELAY_MS));
    }
}

static void prvIMUTask(void *pvParameters)
{
    TickType_t xLastWakeTime;
    bmi160_accel_raw_t raw = {0};
    struct MsgObj msg;   
    SafeUARTprintf("=== BMI160 IMU Task Started (minimal test) ===\r\n");
    vTaskDelay(pdMS_TO_TICKS(500));

    if (!sensorBMI160Init())    
    {
        SafeUARTprintf("!!! BMI160 INIT FAILED - Task stopping !!!\r\n");
        vTaskDelete(NULL);
        return;
    }

    SafeUARTprintf(">>> BMI160 READY - minimal test running <<<\r\n");

    xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        if (xSemaphoreTake(xI2CMutex, pdMS_TO_TICKS(500)) == pdPASS)
        {
            if (sensorBMI160ReadAccel(&raw))
            {
                uint32_t absX = (raw.x < 0) ? (uint32_t)(-raw.x) : (uint32_t)raw.x;
                uint32_t absY = (raw.y < 0) ? (uint32_t)(-raw.y) : (uint32_t)raw.y;
                uint32_t absZ = (raw.z < 0) ? (uint32_t)(-raw.z) : (uint32_t)raw.z;

                g_ui32FilteredBodyAccel = absX + absY + absZ;

                msg.id   = SENSOR_ID_IMU;
                msg.val  = (int32_t)absX;
                msg.avg  = (int32_t)absY;
                msg.val2 = (int32_t)absZ;
                msg.avg2 = (int32_t)g_ui32FilteredBodyAccel;
                msg.flag = 0;

                if (xQueueSend(xQueue, &msg, 0) != pdPASS)
                {
                    xEventGroupSetBits(xEventGroup, EVENT_QUEUE_FULL);
                }
                else
                {
                    xEventGroupSetBits(xEventGroup, EVENT_SENSOR_DATA_READY);
                }
            }

            xSemaphoreGive(xI2CMutex);
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
    }
}

// ====================== OPT3001 Task ======================
static uint32_t UpdateLuxAverage(uint32_t newLux)
{
    // Only divide by the number of valid samples during startup
    if (luxCount < OPT_AVG_WINDOW)
    {
        luxBuffer[luxIndex] = newLux;
        luxSum += newLux;
        luxCount++;
    }
    else
    {
        luxSum -= luxBuffer[luxIndex];
        luxBuffer[luxIndex] = newLux;
        luxSum += newLux;
    }

    luxIndex++;
    if (luxIndex >= OPT_AVG_WINDOW)
    {
        luxIndex = 0;
    }

    return luxSum / luxCount;
}

// ====================== OPT3001 Optical sensor TASK ======================
static void prvOptTask(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint32_t tStart, tEnd, duration_us;
    uint32_t totalTime = 0, sampleCount = 0;
    uint32_t avgTime_us = 0;
    uint16_t rawData = 0;
    float convertedLux = 0.0f;
    bool success = false;

    uint32_t filt_avg10 = 0;

    SafeUARTprintf("=== OPT3001 Task Started ===\r\n");

    if (!sensorOpt3001Init())
    {
        SafeUARTprintf("OPT3001 init FAILED!\r\n");
        vTaskDelete(NULL);
    }

    EnableHighResTimer();

    struct MsgObj xMsgObj = {0};

    for (;;)
    {    
        success = false; // Reset success flag every loop to prevent queuing stale data

        if (xSemaphoreTake(xI2CMutex, pdMS_TO_TICKS(500)) == pdPASS)
        {   
            // high resultion timer function
            tStart = *(volatile uint32_t *)0xE0001004;
            success = sensorOpt3001Read(&rawData);
            tEnd = *(volatile uint32_t *)0xE0001004;
            duration_us = (tEnd - tStart) / (g_ui32SysClock / 1000000UL);

            xSemaphoreGive(xI2CMutex);
        }
       
        if (success)
        {
            totalTime += duration_us;
            sampleCount++;
            avgTime_us = totalTime / sampleCount;

            sensorOpt3001Convert(rawData, &convertedLux);
            
            // Calculate average - now calculates average of available samples before > 10 samples are collected
            uint32_t rawLux = (uint32_t)convertedLux;
            filt_avg10 = UpdateLuxAverage(rawLux);

            xMsgObj.id   = SENSOR_ID_OPT3001; // device sending identifier - replaced magic number with defined ID for OPT3001
            xMsgObj.val  = rawLux;        // Value for unfiltered  data
            xMsgObj.avg  = filt_avg10;          // Value for filtered  data
            xMsgObj.tick = sampleCount;         // leave blank
            xMsgObj.flag = 0;
            xMsgObj.seq  = avgTime_us;          // leave blank

            // === Threshold Events ===
            if (filt_avg10 > g_ui32LuxMaxThreshold)
                xEventGroupSetBits(xEventGroup, EVENT_HIGH_THRESHOLD);
            else if (filt_avg10 < g_ui32LuxMinThreshold) {
                xEventGroupSetBits(xEventGroup, EVENT_LOW_THRESHOLD);
                LEDWrite(LED_D1, LED_D1);          // MIN LUX → LED_D1 ON
            } 
            else 
            {
                
                LEDWrite(LED_D1, 0);               // clear LED when above min
            }

        }
        else
        {         
            success = false;   
            xMsgObj.id   = SENSOR_ID_OPT3001; // Device sending identifier - replaced magic number with defined ID for OPT3001
            xMsgObj.flag = 1;
            xEventGroupSetBits(xEventGroup, EVENT_SENSOR_MISSED);            
        }

        // Send to queue
        if (xQueueSend(xQueue, &xMsgObj, 0) != pdPASS)
        {
            xEventGroupSetBits(xEventGroup, EVENT_QUEUE_FULL);
        }
        // Signal that new data is ready
        xEventGroupSetBits(xEventGroup, EVENT_SENSOR_DATA_READY); // EVENT_BTN_TOGGLE_PLOT should mean button/plot mode, not new sensor data
        
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(200));
    }
}

void prvDisplayTask(void *pvParameters)
{
    //tRectangle sRect;

    FPUEnable();
    FPULazyStackingEnable();

    // Initialize Display
    Kentec320x240x16_SSD2119Init(configCPU_CLOCK_HZ);
    GrContextInit(&sContext, &g_sKentec320x240x16_SSD2119); 

    // Touch + Widgets
    TouchScreenInit(configCPU_CLOCK_HZ);
    TouchScreenCallbackSet(WidgetPointerMessage);

    g_ui32Panel = 0;
    WidgetAdd(WIDGET_ROOT, (tWidget *)g_psPanels);    
    WidgetAdd((tWidget *)&g_psPanels[0], (tWidget *)&g_sTimeCanvas);
    

    WidgetAdd((tWidget *)&g_psPanels[2], (tWidget *)&g_sLightValCanvas);
    WidgetAdd((tWidget *)&g_psPanels[2], (tWidget *)&g_sLightValCanvas2);

    // === Add Checkboxes for Graph Panels ===
    // Panel 2    
    WidgetAdd((tWidget *)&g_psPanels[2], (tWidget *)&g_sPlotModeCheckBox); 
    WidgetAdd((tWidget *)&g_psPanels[2], (tWidget *)&g_sPlotModeCheckBox2);

    // Panel 3
    WidgetAdd((tWidget *)&g_psPanels[3], (tWidget *)&g_sPlotModeCheckBox3);
    WidgetAdd((tWidget *)&g_psPanels[3], (tWidget *)&g_sPlotModeCheckBox4);

    // Panel 4
    WidgetAdd((tWidget *)&g_psPanels[4], (tWidget *)&g_sPlotModeCheckBox6);
    WidgetAdd((tWidget *)&g_psPanels[4], (tWidget *)&g_sPlotModeCheckBox5);

    // Add Motor State and Fault Panel
    WidgetAdd((tWidget *)&g_sDashboard, (tWidget *)&g_sMotorStateCanvas);
    WidgetAdd((tWidget *)&g_sDashboard, (tWidget *)&g_sFaultPanel);  
    WidgetAdd((tWidget *)&g_sDashboard, (tWidget *)&g_sStartButton);

    WidgetAdd((tWidget *)&g_sEngineCanvas, (tWidget *)&g_sCrashTolSlider);
    WidgetAdd((tWidget *)&g_sEngineCanvas, (tWidget *)&g_sAccelMaxSlider);
    WidgetAdd((tWidget *)&g_sEngineCanvas, (tWidget *)&g_sDetRangeSlider);
    WidgetAdd((tWidget *)&g_sEngineCanvas, (tWidget *)&g_sLuxMinSlider);
    // Add permanent side buttons to root (do this only once)
    for (int i = 0; i < 5; i++)
    {
        WidgetAdd(WIDGET_ROOT, (tWidget *)&g_psSideButtons[i]);
    }

    WidgetPaint(WIDGET_ROOT);

    for (;;)
    {
        WidgetMessageQueueProcess();
        vTaskDelay(pdMS_TO_TICKS(20)); 
    }
}

static void prvConfigure(void) 
{
    xQueue = xQueueCreate(mainQUEUE_LENGTH, sizeof(struct MsgObj));
    if (xQueue == NULL)
    {
        SafeUARTprintf("Queue creation FAILED!\r\n");
        return;
    }

    // === CREATE EVENT GROUP ===
    xEventGroup = xEventGroupCreate();
    if (xEventGroup == NULL)
    {
        SafeUARTprintf("Event Group creation FAILED!\r\n");
    }
    else
    {
        SafeUARTprintf("Event Group created successfully.\r\n");
    }

    xTaskCreate(readerTask, "Reader", configMINIMAL_STACK_SIZE * 10, NULL, mainQUEUE_RECEIVE_TASK_PRIORITY , NULL);

}

static void prvConfigureLED( void )
{
    /* Configure initial LED state.  PinoutSet() has already configured LED I/O. */
    LEDWrite(LED_D1, LED_D1);
}

void prvConfigureOPT(void)
{
    // Clear the terminal and print the welcome message.
    SafeUARTprintf("OPT3001 Hardware Setup...\n");

    // The I2C0 peripheral must be enabled before use.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    // Wait until both peripherals are fully clocked
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_I2C0)) {}
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB)) {}

    // Configure the pin muxing for I2C0 on PB2 and PB3.
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);

    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

    // Initialize I2C master
    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), false);

    // IMPORTANT: Give the I2C module time to stabilize
    SysCtlDelay(SysCtlClockGet() / 50);   // ~10 ms delay (safe)

    i2cOptDriverInit();
    IntPrioritySet(INT_I2C0, 0xA0);

    // Enable high-resolution cycle counter for accurate I2C timing 
    EnableHighResTimer();

    SafeUARTprintf("I2C0 and OPT3001 driver initialized (hardware only).\n");

}

static void prvConfigureButton( void )
{
    /* Initialize the LaunchPad Buttons. */
    ButtonsInit();

    /* Configure both switches to trigger an interrupt on a falling edge. */
    GPIOIntTypeSet(BUTTONS_GPIO_BASE, ALL_BUTTONS, GPIO_FALLING_EDGE);

    /* Enable the interrupt for LaunchPad GPIO Port in the GPIO peripheral. */
    GPIOIntEnable(BUTTONS_GPIO_BASE, ALL_BUTTONS);

    /* Enable the Port J interrupt in the NVIC. */
    IntEnable(INT_GPIOJ);

    /* Enable global interrupts in the NVIC. */
    IntMasterEnable();
}

static void prvConfigureHWTimer(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER2)) {}

    TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER2_BASE, TIMER_A, g_ui32SysClock / 5);
    TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);

    IntEnable(INT_TIMER2A);
    TimerEnable(TIMER2_BASE, TIMER_A);
}

void xTimerHandler(void)
{
    TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
    calcTimeCount();
    UpdateTime();
}

// Hall Helper Functions
static bool Motor_ReadHallA(void)
{
    return (GPIOPinRead(HALL_A_GPIO_BASE, HALL_A_GPIO_PIN) != 0);
}

static bool Motor_ReadHallB(void)
{
    return (GPIOPinRead(HALL_BC_GPIO_BASE, HALL_B_GPIO_PIN) != 0);
}

static bool Motor_ReadHallC(void)
{
    return (GPIOPinRead(HALL_BC_GPIO_BASE, HALL_C_GPIO_PIN) != 0);
}

static void Motor_UpdateCommutationFromHall(void)
{
    if (!g_bMotorLibReady)
    {
        return;
    }

    updateMotor(Motor_ReadHallA(),
                Motor_ReadHallB(),
                Motor_ReadHallC());
}

// Hall ISRs
void xHallPortMHandler(void)
{
    uint32_t status = GPIOIntStatus(HALL_A_GPIO_BASE, true);
    GPIOIntClear(HALL_A_GPIO_BASE, status);

    if (status & HALL_A_GPIO_PIN)
    {
        g_ui32HallEdgeCount++;
        g_ui32HallEdgeTotal++;
        Motor_UpdateCommutationFromHall();
    }
}

void xHallPortHHandler(void)
{
    uint32_t status = GPIOIntStatus(HALL_BC_GPIO_BASE, true);
    GPIOIntClear(HALL_BC_GPIO_BASE, status);

    if (status & (HALL_B_GPIO_PIN | HALL_C_GPIO_PIN))
    {
        g_ui32HallEdgeCount++;
        g_ui32HallEdgeTotal++;
        Motor_UpdateCommutationFromHall();
    }
}

static uint32_t Motor_UpdateActualSpeedRPM(void)
{
    uint32_t edges;

    taskENTER_CRITICAL();
    edges = g_ui32HallEdgeCount;
    g_ui32HallEdgeCount = 0;
    taskEXIT_CRITICAL();

    // Speed update period is 10 ms = 100 Hz
    uint32_t rpm = (edges * 60U * 100U) / MOTOR_HALL_EDGES_PER_REV;

    g_ui32MotorActualRPM = rpm;
    g_ui32Speed = (uint16_t)rpm;     // GUI-facing actual RPM

    return rpm;
}

static void Motor_UpdateReferenceRPM(bool estopActive)
{
    int32_t targetRPMx100;
    int32_t currentRPMx100;
    int32_t maxStepRPMx100;

    if (estopActive)
    {
        targetRPMx100 = 0;
        maxStepRPMx100 = (int32_t)(MOTOR_ESTOP_DECEL_RPM_PER_S * 100U * MOTOR_TASK_PERIOD_MS / 1000U);
    }
    else
    {
        targetRPMx100 = (int32_t)(g_ui32DesiredSpeedRPM * 100U);

        if (targetRPMx100 > g_i32ReferenceRPMx100)
        {
            maxStepRPMx100 = (int32_t)(MOTOR_ACCEL_LIMIT_RPM_PER_S * 100U * MOTOR_TASK_PERIOD_MS / 1000U);
        }
        else
        {
            maxStepRPMx100 = (int32_t)(MOTOR_DECEL_LIMIT_RPM_PER_S * 100U * MOTOR_TASK_PERIOD_MS / 1000U);
        }
    }

    currentRPMx100 = g_i32ReferenceRPMx100;

    if (targetRPMx100 > currentRPMx100 + maxStepRPMx100)
    {
        currentRPMx100 += maxStepRPMx100;
    }
    else if (targetRPMx100 < currentRPMx100 - maxStepRPMx100)
    {
        currentRPMx100 -= maxStepRPMx100;
    }
    else
    {
        currentRPMx100 = targetRPMx100;
    }

    if (currentRPMx100 < 0)
    {
        currentRPMx100 = 0;
    }

    g_i32ReferenceRPMx100 = currentRPMx100;
    g_ui32MotorReferenceRPM = (uint32_t)(currentRPMx100 / 100);
}

static void Motor_UpdatePWMController(void)
{
    int32_t referenceRPM = (int32_t)g_ui32MotorReferenceRPM;
    int32_t actualRPM = (int32_t)g_ui32MotorActualRPM;
    int32_t errorRPM = referenceRPM - actualRPM;

    if (referenceRPM <= 0)
    {
        g_i32PIIntegral = 0;
        g_ui16MotorDutyUs = 0;
        setDuty(0);
        g_ui32MotorPWMPercent = 0;
        return;
    }

    g_i32PIIntegral += errorRPM;

    if (g_i32PIIntegral > 60000)
    {
        g_i32PIIntegral = 60000;
    }
    else if (g_i32PIIntegral < -60000)
    {
        g_i32PIIntegral = -60000;
    }

    int32_t ffDuty = MOTOR_MIN_DUTY_US +
        ((referenceRPM * (MOTOR_PWM_PERIOD_US - MOTOR_MIN_DUTY_US)) / MOTOR_MAX_RPM);

    int32_t duty = ffDuty + (errorRPM / 80) + (g_i32PIIntegral / 5000);

    if (duty < 0)
    {
        duty = 0;
    }
    else if (duty > MOTOR_PWM_PERIOD_US)
    {
        duty = MOTOR_PWM_PERIOD_US;
    }

    g_ui16MotorDutyUs = (uint16_t)duty;
    setDuty(g_ui16MotorDutyUs);

    g_ui32MotorPWMPercent = ((uint32_t)g_ui16MotorDutyUs * 100U) / MOTOR_PWM_PERIOD_US;
}

static void prvMotorTask(void *pvParameters)
{
    (void)pvParameters;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    TickType_t lastSpeedUpdate = xLastWakeTime;
    TickType_t lastPlotUpdate = xLastWakeTime;

    uint32_t startingEdgeBaseline = 0;

    SafeUARTprintf("=== Motor Control Task Started ===\r\n");

    if (!initMotorLib(MOTOR_PWM_PERIOD_US))
    {
        SafeUARTprintf("ERROR: initMotorLib failed\r\n");
        g_ui32ActiveFaults |= FAULT_SENSOR;
        SetMotorState(MOTOR_FAULT_LATCHED);
        vTaskDelete(NULL);
        return;
    }

    g_bMotorLibReady = true;

    disableMotor();
    stopMotor(false);
    setDuty(0);

    SafeUARTprintf("desired_rpm,reference_rpm,actual_rpm,pwm_percent,power_w,state,faults\r\n");

    for (;;)
    {
        EventBits_t bits = xEventGroupGetBits(xEventGroup);

        bool estopRequested = ((bits & EVENT_FAULT_ESTOP) != 0U) ||
                              ((g_ui32ActiveFaults & FAULT_EBRAKE) != 0U);

        if ((xTaskGetTickCount() - lastSpeedUpdate) >= pdMS_TO_TICKS(MOTOR_SPEED_UPDATE_PERIOD_MS))
        {
            Motor_UpdateActualSpeedRPM();
            lastSpeedUpdate = xTaskGetTickCount();
        }

        switch (g_eMotorState)
        {
            case MOTOR_IDLE:
            {
                disableMotor();
                stopMotor(false);
                setDuty(0);

                g_i32PIIntegral = 0;
                g_i32ReferenceRPMx100 = 0;
                g_ui32MotorReferenceRPM = 0;
                g_ui32MotorPWMPercent = 0;

                if (g_bMotorStartRequested)
                {
                    g_bMotorStartRequested = false;
                    startingEdgeBaseline = g_ui32HallEdgeTotal;

                    enableMotor();
                    setDuty(MOTOR_START_DUTY_US);
                    Motor_UpdateCommutationFromHall();

                    SetMotorState(MOTOR_STARTING);
                }
                break;
            }

            case MOTOR_STARTING:
            {
                if (estopRequested)
                {
                    SetMotorState(MOTOR_ESTOP_BRAKING);
                    break;
                }

                enableMotor();
                setDuty(MOTOR_START_DUTY_US);
                Motor_UpdateCommutationFromHall();

                if ((g_ui32HallEdgeTotal - startingEdgeBaseline) >= MOTOR_START_FEEDBACK_EDGES)
                {
                    g_i32ReferenceRPMx100 = (int32_t)(g_ui32MotorActualRPM * 100U);
                    SetMotorState(MOTOR_RUNNING);
                }
                break;
            }

            case MOTOR_RUNNING:
            {
                if (estopRequested)
                {
                    SetMotorState(MOTOR_ESTOP_BRAKING);
                    break;
                }

                Motor_UpdateReferenceRPM(false);
                Motor_UpdatePWMController();
                break;
            }

            case MOTOR_ESTOP_BRAKING:
            {
                g_ui32ActiveFaults |= FAULT_EBRAKE;

                Motor_UpdateReferenceRPM(true);
                Motor_UpdatePWMController();

                if ((g_ui32MotorReferenceRPM == 0U) &&
                    (g_ui32MotorActualRPM <= MOTOR_STOPPED_RPM_THRESHOLD))
                {
                    setDuty(0);
                    stopMotor(false);
                    disableMotor();
                    SetMotorState(MOTOR_FAULT_LATCHED);
                }
                break;
            }

            case MOTOR_FAULT_LATCHED:
            {
                setDuty(0);
                stopMotor(false);
                disableMotor();

                if (g_bMotorFaultAckRequested)
                {
                    g_bMotorFaultAckRequested = false;

                    g_ui32ActiveFaults = 0;
                    xEventGroupClearBits(xEventGroup, EVENT_FAULT_ESTOP | EVENT_FAULT_ACK);

                    SetMotorState(MOTOR_IDLE);
                }
                break;
            }

            default:
            {
                SetMotorState(MOTOR_IDLE);
                break;
            }
        }

        if ((xTaskGetTickCount() - lastPlotUpdate) >= pdMS_TO_TICKS(MOTOR_PLOT_PERIOD_MS))
        {
            SafeUARTprintf("%u,%u,%u,%u,%u,%u,0x%02X\r\n",
                           (unsigned int)g_ui32DesiredSpeedRPM,
                           (unsigned int)g_ui32MotorReferenceRPM,
                           (unsigned int)g_ui32MotorActualRPM,
                           (unsigned int)g_ui32MotorPWMPercent,
                           (unsigned int)g_ui32MotorPowerW,
                           (unsigned int)g_eMotorState,
                           (unsigned int)g_ui32ActiveFaults);

            lastPlotUpdate = xTaskGetTickCount();
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(MOTOR_TASK_PERIOD_MS));
    }
}

// Commented out fake fault simulator 
/*
static void prvFaultSimulatorTask(void *pvParameters)
{
    (void)pvParameters;
    
    // Small delay to let display task start first
    vTaskDelay(pdMS_TO_TICKS(1500));
    
    SafeUARTprintf("=== Random Fault Simulator Task Started ===\r\n");
    SafeUARTprintf("Free Heap now: %u bytes\r\n", xPortGetFreeHeapSize());
    
    srand(xTaskGetTickCount());   // Better random seed
    
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        // Toggle faults less frequently
        if ((rand() % 4) == 0)   // ~25% chance
        {
            uint32_t faultToToggle = 1U << (rand() % 4);

            if (g_ui32ActiveFaults & faultToToggle)
            {
                g_ui32ActiveFaults &= ~faultToToggle;
                SafeUARTprintf("FAULT CLEARED: 0x%02X\r\n", (unsigned int)faultToToggle);
            }
            else
            {
                g_ui32ActiveFaults |= faultToToggle;
                SafeUARTprintf("!!! FAULT TRIGGERED: 0x%02X\r\n", (unsigned int)faultToToggle);
            }

            if (g_ui32Panel == 0)
                UpdateFaultPanel();
        }

        // Full clear occasionally
        if ((rand() % 60) == 0)
        {
            g_ui32ActiveFaults = 0;
            SafeUARTprintf("--- ALL FAULTS CLEARED ---\r\n");
            if (g_ui32Panel == 0)
                UpdateFaultPanel();
        }


        // ====================== ENGINE PANEL SENSOR SIMULATION ======================
        g_ui32FilteredBodyAccel = 200 + (rand() % 900);   // varies 200-1100
        g_ui32CurrentDistance   = 30 + (rand() % 300);    // varies 30-330
        g_ui32EngineTemp        = 35 + (rand() % 60);     // varies 35-95 °C

        // Crash / Safe-distance e-stop via sensing subsystem
        if (g_ui32FilteredBodyAccel > g_ui32CrashThreshold ||
            g_ui32CurrentDistance < g_ui32DetRangeVal)
        {
            g_ui32ActiveFaults |= FAULT_EBRAKE;
            SetMotorState(MOTOR_FAULT);
            SafeUARTprintf("!!! E-STOP TRIGGERED (Accel or Distance) !!!\r\n");
        }

        // Heating / Cooling logic - Commented out since its now sensor-driven
        
        
        if (g_ui32EngineTemp > g_ui32HeatingThreshold)
            g_bCoolingOn = true;
        else if (g_ui32EngineTemp < g_ui32CoolingThreshold)
            g_bCoolingOn = false;
        

        // Update Engine Panel live values
        // if (g_ui32Panel == 1)
        // {
        //     WidgetPaint((tWidget *)&g_sEngineCanvas);
        // }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(4000));  // Every 4 seconds
    }
}
*/