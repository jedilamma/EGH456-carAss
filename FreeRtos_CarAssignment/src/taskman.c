#include "taskman.h"


// Motor control backend is implemented in motor_control.c

// Hardware drivers for buttons and LEDs
static void prvOptTask(void *pvParameters);
static void prvLEDTasksw1(void *pvParameters);
static void readerTask(void *pvParameters);
static void prvDisplayTask(void *pvParameters);
static void prvMotorTask(void *pvParameters);
// static void prvFaultSimulatorTask(void *pvParameters);
static void prvIMUTask(void *pvParameters);
static void prvSHT31Task(void *pvParameters); // SHT31 task prototype

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

uint32_t g_plotIMUxHistory[PLOT_HISTORY_SIZE] = {0};
uint32_t g_plotIMUyHistory[PLOT_HISTORY_SIZE] = {0};
uint32_t g_plotIMUzHistory[PLOT_HISTORY_SIZE] = {0};
uint32_t g_plotIMUmagHistory[PLOT_HISTORY_SIZE] = {0};
uint32_t g_plotIMUIndex = 0;

uint32_t g_plotRNGRawHistory[PLOT_HISTORY_SIZE] = {0};
uint32_t g_plotRNGFiltHistory[PLOT_HISTORY_SIZE] = {0};
uint32_t g_plotRNGIndex;

PlotMode_t g_eOPTPlotMode = OPT_PLOT_MODE_RAW_AND_FILTERED;
PlotMode_t g_eIMUPlotMode = IMU_PLOT_MODE_RAW_AND_FILTERED; 
PlotMode_t g_eRNGPlotMode = RNG_PLOT_MODE_RAW_AND_FILTERED;
PlotMode_t g_ePlotMode = RNG_PLOT_MODE_RAW_AND_FILTERED;
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
bool g_bRNGDisplayHold = false;
bool g_bDisplayHold = false;
bool estopRequested = false;
bool g_faultSwitch = false;
bool g_startDisplay = true;
// GUI-facing SHT31 state variables
bool g_bHeatingOn = false;
bool g_bCoolingOn = false;
bool g_bSHT31Valid = false;

// Store temp/humidity as integer x100 to avoid floating point in display code
int32_t  g_i32CabinTempCx100 = 0;
uint32_t g_ui32CabinHumidityRHx100 = 0;

int32_t  g_i32HeatingThresholdCx100 = 1800;
int32_t  g_i32CoolingThresholdCx100 = 2600;

uint16_t g_ui32PowerW = 0;
uint16_t g_ui32Speed = 0; // Gui-facing speed variable (RPM)

// Motor Runtime Globals
volatile uint32_t g_ui32DesiredSpeedRPM = 0;
volatile bool g_bMotorStartRequested = false;
volatile bool g_bMotorStopRequested = false;
volatile bool g_bMotorFaultAckRequested = false;
volatile bool g_bFaultAcknowledged = false;

volatile uint32_t g_ui32MotorReferenceRPM = 0;
volatile uint32_t g_ui32MotorActualRPM = 0;
volatile uint32_t g_ui32MotorPWMPercent = 0;
volatile uint32_t g_ui32MotorPowerW = 0;

tContext sContext;

QueueHandle_t xQueue = NULL;

void vCreateTask(void)
{
    prvConfigureLED();
    prvConfigureButton();
    prvConfigureOPT();
    prvConfigure();   
    prvConfigureHWTimer();         
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
    uint32_t filt_avg10 = 4000;
    int32_t Lastmagnitude = 0;
    (void)pvParameters;

    SafeUARTprintf("Reader Task Started - Draining queue, keeping latest per sensor...\r\n");

    for (;;)
    {
        // Wait for sensor/event updates, but timeout regularly
        uxBits = xEventGroupWaitBits(xEventGroup,
                                     (EVENT_SENSOR_MISSED |
                                      EVENT_QUEUE_FULL |
                                      EVENT_LUXHIGH_THRESHOLD |
                                      EVENT_LUXLOW_THRESHOLD |
                                      EVENT_SENSOR_DATA_READY |
                                      EVENT_DISPLAY_HOLD |
                                      EVENT_MOTOR_STATE_CHANGE | 
                                      EVENT_HIGH_ACCEL_LIMIT),
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

        if (uxBits & EVENT_LUXHIGH_THRESHOLD)
        {
            SafeUARTprintf("EVENT: HIGH THRESHOLD crossed! (Filtered > 1000 lux)\r\n");
        }

        if (uxBits & EVENT_LUXLOW_THRESHOLD)
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
                    
                    filt_avg10 = UpdateAverage(pData->val,SENSOR_ID_OPT3001);     // or UpdateLuxAverage()

                    SafeUARTprintf("RX xQueue id=0x%02x raw=%3d filtered=%3d flag=%u \r\n",
                                    pData->id,
                                    pData->val, filt_avg10,                                                               
                                    pData->flag);  

                    if(filt_avg10 < g_ui32LuxMinThreshold)
                    {
                        xEventGroupSetBits(xEventGroup, EVENT_LUXLOW_THRESHOLD);
                    }
                    if(g_bOPTDisplayHold)
                    {
                        g_plotOPTRawHistory[g_plotOPTIndex] = (uint32_t)pData->val;
                        g_plotOPTFiltHistory[g_plotOPTIndex] = filt_avg10;
                        g_plotOPTIndex = (g_plotOPTIndex + 1) % PLOT_HISTORY_SIZE;

                        if (g_ui32Panel == 2)
                        {
                            WidgetPaint((tWidget *)&g_sCanvas2);
                        }
                    }
                    break;
                }

                case SENSOR_ID_BMI160:
                {
                    if (pData->flag == 1U)
                    {
                        break;
                    }
                    
                    int32_t magnitude = isqrt(pData->val * pData->val + pData->val2 * pData->val2 + pData->avg2 * pData->avg2);

                    SafeUARTprintf("RX xQueue id=0x%02x x=%3d y=%3d z=%3d Mag=%3d flag=%u \r\n",
                                    pData->id,
                                    pData->val,     //x
                                    pData->val2,    //y
                                    pData->avg2,    //z  
                                    magnitude,                
                                    pData->flag);

                    if((Lastmagnitude - magnitude) > 3000)
                    {
                        xEventGroupSetBits(xEventGroup, EVENT_HIGH_ACCEL_LIMIT);
                    }
                    Lastmagnitude = magnitude;

                    if(g_bIMUDisplayHold)
                    {
                        g_plotIMUxHistory[g_plotIMUIndex] = abs(pData->val);
                        g_plotIMUyHistory[g_plotIMUIndex] = abs(pData->val2);
                        g_plotIMUzHistory[g_plotIMUIndex] = abs(pData->avg2);
                        g_plotIMUmagHistory[g_plotIMUIndex] = magnitude;
                        g_plotIMUIndex = (g_plotIMUIndex + 1) % PLOT_HISTORY_SIZE;

                        if (g_ui32Panel == 3)   // updates graph on panel 3
                        {
                            WidgetPaint((tWidget *)&g_sCanvas3);
                        }
                    }
                    break;
                }
                case SENSOR_ID_RANGE:
                {
                    if (pData->flag == 1U)
                    {
                        break;
                    }
                    SafeUARTprintf("RX xQueue id=0x%02x x=%3d y=%3d Mag=%3d flag=%u \r\n",
                                    pData->id,
                                    pData->val,     //Range       
                                    pData->flag);

                    filt_avg10 = UpdateAverage(pData->val,SENSOR_ID_RANGE); 

                    if(g_bRNGDisplayHold)
                    {
                        g_plotOPTRawHistory[g_plotOPTIndex] = (uint32_t)pData->val;
                        g_plotOPTFiltHistory[g_plotOPTIndex] = filt_avg10;
                        g_plotOPTIndex = (g_plotOPTIndex + 1) % PLOT_HISTORY_SIZE;

                        if (g_ui32Panel == 4)   // updates graph on panel 4
                        {
                            WidgetPaint((tWidget *)&g_sCanvas4);
                        }
                    }
                    break;
                }
                case SENSOR_ID_HALL:
                {
                    if (pData->flag == 1U)
                    {
                        break;
                    }
                    SafeUARTprintf("RX xQueue id=0x%02x x=%3d y=%3d Mag=%3d flag=%u \r\n",
                                    pData->id,
                                    pData->val,     //x
                                    pData->val2,    //y
                                    pData->avg2,    //z                  
                                    pData->flag);

                    int32_t magnitude = isqrt(pData->val * pData->val + pData->val2 * pData->val2 + pData->avg2 * pData->avg2);
                    if((Lastmagnitude - magnitude) > 3000)
                    {
                        xEventGroupSetBits(xEventGroup, EVENT_HIGH_ACCEL_LIMIT);
                    }
                    Lastmagnitude = magnitude;

                    g_plotIMUxHistory[g_plotIMUIndex] = abs(pData->val);
                    g_plotIMUyHistory[g_plotIMUIndex] = abs(pData->val2);
                    g_plotIMUmagHistory[g_plotIMUIndex] = magnitude;
                    g_plotIMUIndex = (g_plotIMUIndex + 1) % PLOT_HISTORY_SIZE;

                    if (g_ui32Panel == 3)   // updates graph on panel 3
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
                        g_ui32ActiveFaults &= ~FAULT_OVERTEMP;

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
                        g_ui32ActiveFaults |= FAULT_OVERTEMP;
                        g_bCoolingOn = false;
                        g_bHeatingOn = false;
                    }


                    //instead of this draw when the value changes 
                    //not on every update as it might not change

                    // // Repaint only the active panel
                    // if (g_ui32Panel == 0)
                    // {
                    //     WidgetPaint((tWidget *)&g_sDashboard);
                    // }

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

                msg.id   = SENSOR_ID_BMI160;
                msg.val  = raw.x;     
                msg.val2 = raw.y;
                msg.avg2 = raw.z;   // Using magnitude as main value
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

// ====================== OPT3001 Optical sensor TASK ======================
static void prvOptTask(void *pvParameters)
{
    uint16_t rawData = 0;
    float convertedLux = 0.0f;
    
    struct MsgObj msg = {0};
    TickType_t xLastWakeTime = xTaskGetTickCount();

    SafeUARTprintf("=== OPT3001 Task Started ===\r\n");

    if (xSemaphoreTake(xI2CMutex, pdMS_TO_TICKS(1000)) == pdPASS)
    {
        if (!sensorOpt3001Init())
        {
            SafeUARTprintf("!!! OPT3001 INIT FAILED !!!\r\n");
            xSemaphoreGive(xI2CMutex);
            vTaskDelete(NULL);
            return;
        }
        xSemaphoreGive(xI2CMutex);
        SafeUARTprintf("OPT3001 initialized successfully.\r\n");
    }

    vTaskDelay(pdMS_TO_TICKS(300));   // Wait for first conversion

    for (;;)
    {
        if (xSemaphoreTake(xI2CMutex, pdMS_TO_TICKS(300)) == pdPASS)
        {
            if (sensorOpt3001Read(&rawData))
            {
                sensorOpt3001Convert(rawData, &convertedLux);
                uint32_t rawLux = (uint32_t)convertedLux;
                msg.id   = SENSOR_ID_OPT3001;
                msg.val  = rawLux;
                //msg.avg  = filt_avg10;
                msg.flag = 0;

                // Optional debug
                // SafeUARTprintf("[OPT] Lux=%.1f  Avg=%u\r\n", convertedLux, filt_avg10);
            }
            else
            {
                msg.flag = 1;
            }
            xSemaphoreGive(xI2CMutex);
        }
        else
        {
            msg.flag = 1;
        }

        xQueueSend(xQueue, &msg, 0);
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(200));   // Read every ~200ms
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
    WidgetAdd((tWidget *)&g_sDashboard, (tWidget *)&g_sCanvasDash);    
    WidgetAdd((tWidget *)&g_sDashboard, (tWidget *)&g_sFaultButtonCanvas);

    WidgetAdd((tWidget *)&g_sFaultButtonCanvas, (tWidget *)&g_sClearFaultButton);
    g_bFaultButtonVisible = true;   // initial state

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

static void prvMotorTask(void *pvParameters)
{
    TickType_t xLastWakeTime;
    MotorStatus_t xMotorStatus;
    MotorControlFault_t eFaultReason;
    uint32_t ui32RequestedSpeed;
    bool bExternalFaultActive;

    (void)pvParameters;

    SafeUARTprintf("=== Motor Task Started ===\r\n");

    Motor_Init();

    if (!Motor_IsReady())
    {
        SafeUARTprintf("ERROR: Motor backend failed to initialise\r\n");
        g_ui32ActiveFaults |= FAULT_MOTOR_SENSOR;
        SetMotorState(MOTOR_FAULT_LATCHED);
    }

    xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        ui32RequestedSpeed = g_ui32DesiredSpeedRPM;

        bExternalFaultActive =
            ((g_ui32ActiveFaults & (FAULT_EBRAKE | FAULT_OVERCURRENT)) != 0U) ||
            (g_eMotorState == MOTOR_ESTOP_BRAKING) ||
            (g_eMotorState == MOTOR_FAULT_LATCHED);

        eFaultReason = MOTOR_CONTROL_FAULT_MANUAL_ESTOP;

        if ((g_ui32ActiveFaults & FAULT_OVERCURRENT) != 0U)
        {
            eFaultReason = MOTOR_CONTROL_FAULT_POWER;
        }
        else if (g_ui32FilteredBodyAccel > g_ui32CrashThreshold)
        {
            eFaultReason = MOTOR_CONTROL_FAULT_ACCELERATION;
        }
        else if (g_ui32CurrentDistance < g_ui32DetRangeVal)
        {
            eFaultReason = MOTOR_CONTROL_FAULT_DISTANCE;
        }

        if (g_bMotorFaultAckRequested || g_bFaultAcknowledged)
        {
            g_bMotorFaultAckRequested = false;
            g_bFaultAcknowledged = false;

            Motor_AcknowledgeFault();
            g_ui32ActiveFaults = 0;
        }
        else if (bExternalFaultActive)
        {
            Motor_EStop(eFaultReason);
        }
        else
        {
            Motor_SetSpeed((float)ui32RequestedSpeed);

            if (g_bMotorStartRequested)
            {
                g_bMotorStartRequested = false;

                if (ui32RequestedSpeed > 0U)
                {
                    Motor_Start();
                }
            }

            if (g_bMotorStopRequested || ui32RequestedSpeed == 0U)
            {
                g_bMotorStopRequested = false;
                Motor_Stop();
            }
            else if ((g_eMotorState == MOTOR_STARTING) ||
                     (g_eMotorState == MOTOR_RUNNING))
            {
                Motor_Start();
            }
        }

        Motor_ControlStep(MOTOR_CONTROL_PERIOD_MS);
        xMotorStatus = Motor_GetStatus();

        g_ui32MotorReferenceRPM = (uint32_t)xMotorStatus.reference_speed_rpm;
        g_ui32MotorActualRPM = (uint32_t)xMotorStatus.actual_speed_rpm;
        g_ui32MotorPWMPercent = (uint32_t)xMotorStatus.pwm_duty_percent;
        g_ui32MotorPowerW = (uint32_t)xMotorStatus.motor_power_w;
        g_ui32Speed = (uint16_t)xMotorStatus.actual_speed_rpm;

        switch (xMotorStatus.state)
        {
            case MOTOR_CONTROL_STATE_STARTING:
                if (g_eMotorState != MOTOR_STARTING)
                {
                    SetMotorState(MOTOR_STARTING);
                }
                break;

            case MOTOR_STARTING:
                    SafeUARTprintf("Motor STARTING...\r\n");
                    vTaskDelay(pdMS_TO_TICKS(2000));        // Starting sequence
                    SetMotorState(MOTOR_RUNNING);
                break;

            case MOTOR_RUNNING:
                SafeUARTprintf("Motor RUNNING - Normal operation\r\n");
                vTaskDelay(pdMS_TO_TICKS(8000));        // Run for 8 seconds
                
                // Random chance to create a fault (for demo)
                if ((rand() % 20) == 0)                  // ~5% chance
                    SetMotorState(MOTOR_FAULT_LATCHED);
                else
                    SetMotorState(MOTOR_IDLE);          // Back to idle
                break;

            case MOTOR_FAULT_LATCHED:
                // SafeUARTprintf("!!! MOTOR FAULT !!!\r\n");
                // g_ui32ActiveFaults |= FAULT_EBRAKE | FAULT_OVERCURRENT;
                // vTaskDelay(pdMS_TO_TICKS(5000));        // Stay in fault for 5s
                
                // // Auto recover for demo
                // g_ui32ActiveFaults = 0;
                // SetMotorState(MOTOR_IDLE);


                SafeUARTprintf("!!! MOTOR FAULT !!!\r\n");
                g_ui32ActiveFaults |= FAULT_EBRAKE | FAULT_OVERCURRENT;
                g_bFaultAcknowledged = false;

                // PAUSE motor task until user presses ACK FAULT button
                while (g_eMotorState == MOTOR_FAULT_LATCHED && !g_bFaultAcknowledged)
            {
                if (estopRequested)
                {
                    SetMotorState(MOTOR_RUNNING);
                }
                break;

            case MOTOR_CONTROL_STATE_ESTOP_BRAKING:
                g_ui32ActiveFaults |= FAULT_EBRAKE;
                if (g_eMotorState != MOTOR_ESTOP_BRAKING)
                {
                    SetMotorState(MOTOR_ESTOP_BRAKING);
                }
                break;

            case MOTOR_CONTROL_STATE_FAULT_LATCHED:
                g_ui32ActiveFaults |= FAULT_EBRAKE;
                if (g_eMotorState != MOTOR_FAULT_LATCHED)
                {
                    SetMotorState(MOTOR_FAULT_LATCHED);
                }
                break;

            case MOTOR_CONTROL_STATE_IDLE:
            default:
                if (g_eMotorState != MOTOR_IDLE)
                {
                    SetMotorState(MOTOR_IDLE);
                }
                break;
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(MOTOR_CONTROL_PERIOD_MS));
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
        if ((rand() % 4) == 0)
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
            SetMotorState(MOTOR_FAULT_LATCHED);
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