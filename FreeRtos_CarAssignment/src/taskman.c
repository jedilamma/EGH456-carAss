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
static void prvVL53Task(void *pvParameters);
static void prvHallTask(void *pvParameters);

static void prvConfigureLED(void);
static void prvConfigureButton(void);
static void prvConfigurei2c(void);
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
uint32_t g_ui32DetRangeVal           = 60;    // configurable safe distance
uint32_t g_ui32LuxMinThreshold       = 10;    // low lux trigger
uint32_t g_ui32LuxMaxThreshold       = 950;   // high lux trigger
uint32_t g_ui32EngineTemp            = 55;    // simulated engine temp for heating/cooling
uint32_t g_ui32HeatingThreshold      = 100;
uint32_t g_ui32CoolingThreshold      = 10;
uint32_t g_ui32AccelMaxVal           = 10;
uint32_t g_ui32CrashLevels           = 10;
uint32_t g_ui32MinSafeDistance       = 60;
uint32_t Hfilt_avg10 = 0;
int32_t Tfilt_avg10 = 0;
// Already declared in taskman.h as extern
volatile uint32_t g_ui32TimeStamp = 0;
volatile uint8_t  uiIndex = 0;
volatile uint8_t  uiIndex_2 = 0;
volatile uint32_t g_pui32ButtonPressed = 0;

volatile MotorState_t g_eMotorState = MOTOR_IDLE;

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
uint32_t g_plotRNGIndex = 0;

// Dashboard speed history (for small graph)
uint32_t g_plotDashSpeedHistory[PLOT_LARGE_HISTORY_SIZE] = {0};
uint32_t g_plotDashAvgHistory[PLOT_LARGE_HISTORY_SIZE] = {0};
uint32_t g_plotDashIndex = 0;

PlotMode_t g_eOPTPlotMode = OPT_PLOT_MODE_RAW_AND_FILTERED;
PlotMode_t g_eIMUPlotMode = IMU_PLOT_MODE_RAW_AND_FILTERED; 
PlotMode_t g_eRNGPlotMode = RNG_PLOT_MODE_RAW_AND_FILTERED;
PlotMode_t g_ePlotMode = RNG_PLOT_MODE_RAW_AND_FILTERED;

// Moving average state for OPT3001 - Originally average divided by 10 even before 10 samples existed, 
// leading to inaccurate initial readings

bool g_bOPTDisplayHold = true;
bool g_bIMUDisplayHold = true;
bool g_bRNGDisplayHold = true;
bool g_bDisplayHold = true;

bool estopRequested = false;
bool g_faultSwitch = false;
bool g_startDisplay = true;
// GUI-facing SHT31 state variables
bool g_bHeatingOn = false;
bool g_bCoolingOn = false;
bool g_bSHT31Valid = false;

bool g_ePrintMode = false;
// Store temp/humidity as integer x100 to avoid floating point in display code
int32_t  g_i32CabinTempCx100 = 0;
uint32_t g_ui32CabinHumidityRHx100 = 0;

int32_t  g_i32HeatingThresholdCx100 = 1800;
int32_t  g_i32CoolingThresholdCx100 = 2600;

uint16_t g_ui32PowerW = 0;
uint16_t g_ui32RCVSpeed = 0; // Gui-facing speed variable (RPM)

// Motor Runtime Globals
volatile int g_ui32DesiredSpeedRPM = 0;
volatile bool g_bMotorStartRequested = false;
volatile bool g_bMotorStopRequested = false;
volatile bool g_bMotorFaultAckRequested = false;
bool g_bFaultAcknowledged = false;

volatile int g_ui32MotorReferenceRPM = 0;
volatile int g_ui32MotorActualRPM = 0;
volatile int g_ui32MotorPWMPercent = 0;
volatile int g_ui32MotorPowerW = 0;

tContext sContext;

QueueHandle_t xQueue = NULL;

void vCreateTask(void)
{
    prvConfigureLED();
    prvConfigureButton();
    prvConfigurei2c();
    prvConfigure();   
    prvConfigureHWTimer();         
    SetTimeFromCompileTime();
    

    // ============================================================
    xTaskCreate(readerTask,         "Reader", configMINIMAL_STACK_SIZE * 10, NULL, mainQUEUE_RECEIVE_TASK_PRIORITY , NULL);

    xTaskCreate(prvLEDTasksw1,      "LED",     configMINIMAL_STACK_SIZE * 2, NULL, tskIDLE_PRIORITY + 2, NULL);
    xTaskCreate(prvIMUTask,         "IMU",     configMINIMAL_STACK_SIZE * 6, NULL, tskIDLE_PRIORITY + 2, NULL);     
    xTaskCreate(prvOptTask,         "OPT",     configMINIMAL_STACK_SIZE * 6, NULL, tskIDLE_PRIORITY + 2, NULL);    
    xTaskCreate(prvDisplayTask,     "DISPLAY", configMINIMAL_STACK_SIZE * 12, NULL, tskIDLE_PRIORITY + 3, NULL);  // biggest
    //xTaskCreate(prvFaultSimulatorTask, "FAULTSIM", configMINIMAL_STACK_SIZE * 4, NULL, tskIDLE_PRIORITY + 2, NULL);
    xTaskCreate(prvMotorTask,       "MOTOR",    configMINIMAL_STACK_SIZE * 6, NULL, tskIDLE_PRIORITY + 3, NULL);
    xTaskCreate(prvSHT31Task,       "SHT31",    configMINIMAL_STACK_SIZE * 4, NULL, tskIDLE_PRIORITY + 2, NULL); // Create the SHT31 sampling/filtering task 
    xTaskCreate(prvVL53Task,        "VL53",     configMINIMAL_STACK_SIZE * 8, NULL, tskIDLE_PRIORITY + 2, NULL);
    xTaskCreate(prvHallTask,        "HALL", configMINIMAL_STACK_SIZE * 8, NULL, tskIDLE_PRIORITY + 3, NULL);


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
        }
    }
}

// New FreeRTOS task for the VL53 Time of flight range sensor
static void prvVL53Task(void *pvParameters)
{
    uint16_t distance = 0;
    struct MsgObj msg = {0};
    TickType_t xLastWakeTime = xTaskGetTickCount();

    SafeUARTprintf("=== VL53L0X Task Started ===\r\n");

    for (;;)
    {
        if (xSemaphoreTake(xI2C2Mutex, pdMS_TO_TICKS(200)) == pdPASS)
        {
            if (VL53_TestSingleShot(&distance))
            {
                msg.id   = SENSOR_ID_RANGE;
                msg.val  = distance;      // in mm
                msg.flag = 0;
            }
            else
            {
                msg.flag = 1;
            }
            xSemaphoreGive(xI2C2Mutex);
        }
        else
        {
            msg.flag = 1;
        }

        xQueueSend(xQueue, &msg, 0);
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));
    }
}

// ====================== HALL SENSOR TASK ======================
static void prvHallTask(void *pvParameters)
{
    (void)pvParameters;

    TickType_t xLastWakeTime = xTaskGetTickCount();

    SafeUARTprintf("=== Hall Sensor Task Started ===\r\n");

    for (;;)
    {
        if (g_eMotorState == MOTOR_RUNNING || g_eMotorState == MOTOR_STARTING || g_eMotorState == MOTOR_ESTOP_BRAKING)
        {
            if (Motor_IsReady())
            {
                MotorStatus_t mstatus = Motor_GetStatus();

                struct MsgObj msg = {0};           // Declare inside the if

                msg.id    = SENSOR_ID_HALL;
                msg.flag  = 0;
                msg.val   = Motor_GetCurrentHallState();
                msg.avg   = mstatus.hall_edge_count;
                msg.val2  = (int32_t)mstatus.actual_speed_rpm;

                // Send duty as integer ×10 for accurate 1 decimal place printing
                msg.avg2  = (int32_t)(mstatus.pwm_duty_percent * 10);

                // Keep float versions too (useful for other things)
                msg.fval  = mstatus.pwm_duty_percent;
                msg.favg  = mstatus.motor_power_w;
                msg.tick  = xTaskGetTickCount();

                if (xQueueSend(xQueue, &msg, 0) != pdPASS)
                {
                    xEventGroupSetBits(xEventGroup, EVENT_QUEUE_FULL);
                }
                else
                {
                    xEventGroupSetBits(xEventGroup, EVENT_SENSOR_DATA_READY);
                }
            }
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50));
    }
}

// New FreeRTOS task for the SHT31 temperature/humidity sensor
static void prvSHT31Task(void *pvParameters)
{
    (void)pvParameters;

    float temperature = 0.0f;
    float humidity = 0.0f;
    struct MsgObj xMsgObj = {0};
    TickType_t xLastWakeTime = xTaskGetTickCount();

    SafeUARTprintf("=== SHT31 Task (I2C2 shared) Started ===\r\n");

    for (;;)
    {
        bool success = false;

        if (xSemaphoreTake(xI2C2Mutex, pdMS_TO_TICKS(400)) == pdPASS)
        {
            success = SHT31_Read(&temperature, &humidity);
            xSemaphoreGive(xI2C2Mutex);
        }

        // Send data through the same queue
        xMsgObj.id   = SENSOR_ID_SHT31;
        xMsgObj.tick = xTaskGetTickCount();

        if (success)
        {
            xMsgObj.val  = FloatToX100(temperature);                 // filtered later if needed
            xMsgObj.val2 = (int32_t)FloatToUX100(humidity);
            xMsgObj.flag = 0;
            g_bSHT31Valid = true;
        }
        else
        {
            xMsgObj.flag = 1;
            g_bSHT31Valid = false;
        }

        xQueueSend(xQueue, &xMsgObj, 0);
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(800));   // 1 Hz is fine for temp/humidity
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
    uint32_t filt_avg10 = 0;
    //int diffmag = 0;
    //static uint32_t Hfilt_avg10 = 0;
    static uint32_t Cfilt_avg10 = 0;
    //int32_t Lastmagnitude = 4000;
    (void)pvParameters;

    SafeUARTprintf("Reader Task Started - Draining queue, keeping latest per sensor...\r\n");

    for (;;)
    {

            // Clear all latched faults here
        // g_ui32ActiveFaults &= ~(FAULT_EBRAKE | FAULT_OVERCURRENT | 
        //                         FAULT_ACCEL_SENSOR | 
        //                         FAULT_RANGE_SENSOR);

        // Wait for sensor/event updates, but timeout regularly
        uxBits = xEventGroupWaitBits(xEventGroup,
                                     (EVENT_SENSOR_MISSED |
                                      EVENT_QUEUE_FULL |
                                      EVENT_LUXHIGH_THRESHOLD |
                                      EVENT_LUXLOW_THRESHOLD |
                                      EVENT_SENSOR_DATA_READY |
                                      EVENT_DISPLAY_HOLD |
                                      EVENT_MOTOR_STATE_CHANGE | 
                                      EVENT_HIGH_ACCEL_LIMIT |
                                      EVENT_FAULT_RANGE),
                                     pdTRUE,
                                     pdFALSE,
                                     pdMS_TO_TICKS(100));

        if (uxBits & EVENT_MOTOR_STATE_CHANGE)
        {
            // if (g_eMotorState == MOTOR_CONTROL_STATE_ESTOP_BRAKING || g_eMotorState == MOTOR_CONTROL_STATE_FAULT_LATCHED)
            // {
            //     g_ui32ActiveFaults |= FAULT_EBRAKE;
            // }
            // else
            // {
            //     g_ui32ActiveFaults &= ~FAULT_EBRAKE;
            // }
        }

        if (uxBits & EVENT_QUEUE_FULL)
        {
            SafeUARTprintf("EVENT: QUEUE FULL! Message dropped.\r\n");
        }

        if (uxBits & EVENT_LUXHIGH_THRESHOLD)
        {
            SafeUARTprintf("EVENT: HIGH THRESHOLD crossed! (Filtered > 1000 lux)\r\n");
        }

        if(uxBits & EVENT_FAULT_RANGE)
        {
            static char buff1[50];
            usprintf(buff1,"EVENT: CLOSE DETECTION (Filtered < %d mm)\r\n",g_ui32CrashThreshold);

        }
        if(uxBits & EVENT_HIGH_ACCEL_LIMIT)
        {
            static char buff1[50];
            usprintf(buff1,"EVENT: HIGH ACCEL LEVELS (Filtered < %d)\r\n",g_ui32CrashThreshold);

        }
        static uint8_t lighttimer = 0;
        if(uxBits & EVENT_LUXLOW_THRESHOLD)
        {
            static char buff1[50];
            //usprintf(buff1,"EVENT: LUXLOW THRESHOLD (Filtered < %d lux)\r\n",g_ui32LuxMinThreshold);
            SafeUARTprintf(buff1);            
            //if (!uiIndex)  LEDWrite(LED_D1, LED_D1);
            LEDWrite(LED_D1, LED_D1);
            lighttimer = 10;
        }
        else
        {   
            if(lighttimer > 1)
            {
                lighttimer--;
            }
            else
            {
                LEDWrite(LED_D1|LED_D2|LED_D3|LED_D4, 0);
            }
            
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

                    if(g_ePrintMode)
                    {
                        SafeUARTprintf("RX xQueue id=0x%02x raw=%3d filtered=%3d flag=%u \r\n",
                                                            pData->id,
                                                            pData->val, filt_avg10,                                                               
                                                            pData->flag); 
                    }
                     

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
                    if(g_ePrintMode)
                    {   
                        SafeUARTprintf("RX xQueue id=0x%02x x=%3d y=%3d z=%3d Mag=%3d flag=%u \r\n",
                                        pData->id,
                                        pData->val,     //x
                                        pData->val2,    //y
                                        pData->avg2,    //z  
                                        magnitude,                
                                        pData->flag);
                    }
                   // diffmag = abs(Lastmagnitude - magnitude);
                    if(magnitude > 4700 && (g_eMotorState == MOTOR_RUNNING || g_eMotorState == MOTOR_STARTING))
                    {   
                        g_ui32CrashLevels = magnitude;
                        g_ui32ActiveFaults |= FAULT_ACCEL_SENSOR;
                        if (g_ui32Panel == 0) 
                        {
                            WidgetPaint((tWidget *)&g_sFaultPanel);
                        }
                    }
                    //Lastmagnitude = magnitude;

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
                    if(g_ePrintMode)
                    {
                        SafeUARTprintf("RX xQueue id=0x%02x x=%3d flag=%u \r\n",
                                    pData->id,
                                    pData->val,     //Range       
                                    pData->flag);
                    }

                    filt_avg10 = UpdateAverage(pData->val,SENSOR_ID_RANGE); 
                    if(filt_avg10 < g_ui32MinSafeDistance  && (g_eMotorState == MOTOR_RUNNING || g_eMotorState == MOTOR_STARTING))
                    {   
                        g_ui32CurrentDistance = filt_avg10;
                        g_ui32ActiveFaults |= FAULT_RANGE_SENSOR;
                        if (g_ui32Panel == 0) 
                        {
                            WidgetPaint((tWidget *)&g_sFaultPanel);
                        }
                    }
                    if(g_bRNGDisplayHold)
                    {
                        g_plotRNGRawHistory[g_plotRNGIndex] = (uint32_t)pData->val;
                        g_plotRNGFiltHistory[g_plotRNGIndex] = filt_avg10;
                        g_plotRNGIndex = (g_plotRNGIndex + 1) % PLOT_HISTORY_SIZE;

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
                        break;

                    static uint8_t counter = 0;
                    Hfilt_avg10 = UpdateAverage((unsigned int)pData->val2, SENSOR_ID_HALL); 
                    Cfilt_avg10 = UpdateAverage((uint32_t)pData->favg, 67U);
                    // === Update all motor GUI variables from queue ===
                    g_ui32RCVSpeed          = Hfilt_avg10; // send to gui to print
                    g_ui32MotorActualRPM    = pData->val2;
                    g_ui32MotorPWMPercent   = pData->avg2;
                    g_ui32MotorPowerW       = Cfilt_avg10;     // Power from Hall queue
                    // Feed dashboard speed graph

                    if (g_ui32MotorPowerW > 40 &&
                        (g_eMotorState == MOTOR_STARTING || g_eMotorState == MOTOR_RUNNING))
                    {
                        g_ui32ActiveFaults |= FAULT_OVERCURRENT;
                        if (g_ui32Panel == 0) 
                        {
                            WidgetPaint((tWidget *)&g_sFaultPanel);
                        }
                    }
                    
                    g_plotDashSpeedHistory[g_plotDashIndex] = (unsigned int)pData->val2;
                    g_plotDashAvgHistory[g_plotDashIndex] = Hfilt_avg10;
                    g_plotDashIndex = (g_plotDashIndex + 1) % PLOT_LARGE_HISTORY_SIZE; 

                    if (counter > 5) // about 5*50ms
                    {
                        SafeUARTprintf("RX xQueue HALL: id=0x%02x state=0x%02x edges=%u speed=%d AVG=%3u ref speed %3d rpm duty=%d.%d%%\r\n",
                                    pData->id,
                                    (unsigned)pData->val,
                                    (unsigned)pData->avg,
                                    (int)pData->val2,
                                    Hfilt_avg10,g_ui32DesiredSpeedRPM,
                                    (int)(pData->avg2 / 10),      // whole part
                                    (int)(pData->avg2 % 10));     // decimal part
                        counter = 0;
                    }
                    else
                    {
                        counter++;
                    }

                    // Safety checks
                    uint8_t hallState = (uint8_t)pData->val;
                    if (hallState == 0x00 || hallState == 0x07)
                    {
                        if (g_eMotorState == MOTOR_RUNNING || g_eMotorState == MOTOR_STARTING)
                        {
                            SafeUARTprintf("HALL: Invalid state -> E-STOP\r\n");
                            Motor_Brake();
                            SetMotorState(MOTOR_ESTOP_BRAKING);
                        }
                    }

                    break;
                }
                case SENSOR_ID_SHT31:
                {
                    // Extract SHT31 filtered values from the message
                    int32_t temp_x100 = (int32_t)pData->val;
                    uint32_t hum_x100 = (uint32_t)pData->val2;
                    if(temp_x100/100 < 100 && temp_x100/100 > -40)
                        Tfilt_avg10 = UpdateAverage(temp_x100, SENSOR_ID_SHT31);
                    // Update GUI-facing globals
                    g_i32CabinTempCx100 = Tfilt_avg10;
                    g_ui32CabinHumidityRHx100 = hum_x100;
                    g_bSHT31Valid = (pData->flag == 0U);


                    if (g_bSHT31Valid)
                    {
                        //g_ui32ActiveFaults &= ~FAULT_OVERTEMP;

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
                        //g_ui32ActiveFaults |= FAULT_OVERTEMP;
                        g_bCoolingOn = false;
                        g_bHeatingOn = false;
                    }


                    //instead of this draw when the value changes 
                    //not on every update as it might not change

                    // Repaint always
                    //UpdateTemp();

                    if (g_ui32Panel == 5)           // ← Now updates Panel 6 (Temp panel)
                    {
                        WidgetPaint((tWidget *)&g_sCanvas5);
                    }

                    break;
                }

                default:
                {
                    break;
                }
            }
        }
        if (g_eMotorState == MOTOR_STARTING || g_eMotorState == MOTOR_RUNNING)
        {
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
    vTaskDelay(pdMS_TO_TICKS(200));

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
        if (xSemaphoreTake(xI2C2Mutex, pdMS_TO_TICKS(200)) == pdPASS)
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

            xSemaphoreGive(xI2C2Mutex);
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(25));
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

    if (xSemaphoreTake(xI2C2Mutex, pdMS_TO_TICKS(1000)) == pdPASS)
    {
        if (!sensorOpt3001Init())
        {
            SafeUARTprintf("!!! OPT3001 INIT FAILED !!!\r\n");
            xSemaphoreGive(xI2C2Mutex);
            vTaskDelete(NULL);
            return;
        }
        xSemaphoreGive(xI2C2Mutex);
        SafeUARTprintf("OPT3001 initialized successfully.\r\n");
    }

    vTaskDelay(pdMS_TO_TICKS(300));   // Wait for first conversion

    for (;;)
    {
        if (xSemaphoreTake(xI2C2Mutex, pdMS_TO_TICKS(300)) == pdPASS)
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
            xSemaphoreGive(xI2C2Mutex);
        }
        else
        {
            msg.flag = 1;
        }

        xQueueSend(xQueue, &msg, 0);
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(500));   // Read every ~200ms
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
    //WidgetAdd((tWidget *)&g_psPanels[0], (tWidget *)&g_sTempCanvas);

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
    //WidgetAdd((tWidget *)&g_sDashboard, (tWidget *)&g_sCanvasDash);    
    WidgetAdd((tWidget *)&g_sDashboard, (tWidget *)&g_sFaultButtonCanvas);
    // Add the new blue canvas to the dashboard
    //WidgetAdd((tWidget *)&g_sDashboard, (tWidget *)&g_sDashboardBlueCanvas);

    WidgetAdd((tWidget *)&g_sFaultButtonCanvas, (tWidget *)&g_sClearFaultButton);
    g_bFaultButtonVisible = true;   // initial state

    WidgetAdd((tWidget *)&g_sEngineCanvas, (tWidget *)&g_sCrashTolSlider);
    WidgetAdd((tWidget *)&g_sEngineCanvas, (tWidget *)&g_sAccelMaxSlider);
    WidgetAdd((tWidget *)&g_sEngineCanvas, (tWidget *)&g_sDetRangeSlider);
    WidgetAdd((tWidget *)&g_sEngineCanvas, (tWidget *)&g_sLuxMinSlider);


    // Add permanent side buttons to root (do this only once)
    for (int i = 0; i < 6; i++)
    {
        WidgetAdd(WIDGET_ROOT, (tWidget *)&g_psSideButtons[i]);        
    }
   
    WidgetPaint(WIDGET_ROOT);    

    for (;;)
    {
        WidgetMessageQueueProcess();

        // Update the blue dashboard area directly (no WidgetPaint)
        if (g_ui32Panel == 0)
        {
                tContext *ctx = &sContext;
                // === RPM Value ===
                GrContextFontSet(ctx, &g_sFontCm20);
                GrContextForegroundSet(ctx, ClrCyan);
                char buf[32];
                usprintf(buf, "RPM: %d", g_ui32RCVSpeed);
                GrStringDraw(ctx, buf, -1, 45, 40, 1);

                // === Watts Value ===
                GrContextFontSet(ctx, &g_sFontCm16);
                GrContextForegroundSet(ctx, ClrLightGrey);
                usprintf(buf, "Power: %d W", g_ui32MotorPowerW);
                GrStringDraw(ctx, buf, -1, 45, 65, 1);

                // === RPM setting Value ===
                GrContextFontSet(ctx, &g_sFontCm12);
                GrContextForegroundSet(ctx, ClrWhite);
                usprintf(buf, "%u", g_ui32DesiredSpeedRPM);
                GrStringDraw(ctx, buf, -1, 5, 200, 1);

                // === RPM 
                GrContextFontSet(ctx, &g_sFontCm12);
                GrContextForegroundSet(ctx, ClrWhite);
                GrStringDraw(ctx, "RPM", -1, 10, 30, 1);

                 // === temp

                if (g_i32CabinTempCx100/100 > 0 && g_i32CabinTempCx100/100 < 100)
                { 
                    GrContextFontSet(ctx, &g_sFontCm16);
                    GrContextForegroundSet(ctx, ClrWhite);
                    usprintf(buf, "%u C", (uint32_t)Tfilt_avg10/100);
                    GrStringDraw(ctx, buf, -1, 250, 15, 1);
                }
                


            switch(g_eMotorState)
            {   case MOTOR_STARTING:
                case MOTOR_ESTOP_BRAKING:
                case MOTOR_RUNNING: 
                case MOTOR_FAULT_LATCHED:
                    DrawDashboardGraphAndValues();
                break;
                default:
                break;
            }
        }


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


}

static void prvConfigureLED( void )
{
    /* Configure initial LED state.  PinoutSet() has already configured LED I/O. */
    LEDWrite(LED_D1, LED_D1);
}

void prvConfigurei2c(void)
{
// // === I2C0 (keep if SHT31/VL53 or anything else still uses it) ===
//     SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
//     SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
//     while (!SysCtlPeripheralReady(SYSCTL_PERIPH_I2C0)) {}
//     while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB)) {}

//     GPIOPinConfigure(GPIO_PB2_I2C0SCL);
//     GPIOPinConfigure(GPIO_PB3_I2C0SDA);
//     GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
//     GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);
//     I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), false);
//     SysCtlDelay(SysCtlClockGet() / 50);

    // === I2C2 - NEW for BMI160 + OPT3001 on BoosterPack 2 ===
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C2);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_I2C2)) {}
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPION)) {}

    GPIOPinConfigure(GPIO_PN5_I2C2SCL);
    GPIOPinConfigure(GPIO_PN4_I2C2SDA);
    GPIOPinTypeI2CSCL(GPIO_PORTN_BASE, GPIO_PIN_5);
    GPIOPinTypeI2C(GPIO_PORTN_BASE, GPIO_PIN_4);
    I2CMasterInitExpClk(I2C2_BASE, SysCtlClockGet(), false);
    SysCtlDelay(SysCtlClockGet() / 50);

    i2cOptDriverInit();          // still needed for semaphore etc.
    IntPrioritySet(INT_I2C0, 0xA0);   // keep if I2C0 is still used

    SafeUARTprintf("I2C0 + I2C2 configured for BoosterPack 2 (I2C2 on PN4/PN5)\r\n");

    i2cOptDriverInit();
    IntPrioritySet(INT_I2C0, 0xA0);

    // Enable high-resolution cycle counter for accurate I2C timing 
    EnableHighResTimer();

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
    (void)pvParameters;

    MotorStatus_t motorStatus;
    bool externalFaultActive = false;
    static float s_fCommandedRPM = 0.0f;   // Internal ramp target (manual control)

    const float ACCEL_RATE_RPM_PER_S      = 500.0f;
    const float EBRAKE_DECEL_RATE_RPM_PER_S = 1000.0f;
    const float DT_SECONDS = (float)MOTOR_CONTROL_PERIOD_MS / 1000.0f;

    SafeUARTprintf("=== Motor Task Started (Manual Ramp + Controlled E-Brake) ===\r\n");

    Motor_Init();

    if (!Motor_IsReady())
    {
        SafeUARTprintf("ERROR: Motor driver failed to initialise\r\n");
        g_ui32ActiveFaults |= FAULT_MOTOR_SENSOR;
        SetMotorState(MOTOR_FAULT_LATCHED);
    }

    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        motorStatus = Motor_GetStatus();

        externalFaultActive =
            ((g_ui32ActiveFaults & (FAULT_EBRAKE | FAULT_OVERCURRENT)) != 0) ||
            (g_ui32FilteredBodyAccel > g_ui32CrashThreshold) ||
            (g_ui32CurrentDistance < g_ui32DetRangeVal);

        switch (g_eMotorState)
        {
            case MOTOR_IDLE:
            {
                Motor_SetDutyPercent(0.0f);
                Motor_DisableCommutation();
                s_fCommandedRPM = 0.0f;

                if (g_bMotorStartRequested)
                {
                    g_bMotorStartRequested = false;
                    SafeUARTprintf("Motor: Armed -> STARTING\r\n");
                    SetMotorState(MOTOR_STARTING);
                }
                break;
            }

            case MOTOR_STARTING:
            {
                if (externalFaultActive)
                {
                    Motor_StopOpenLoopStartup();
                    SetMotorState(MOTOR_ESTOP_BRAKING);
                    break;
                }

                Motor_Enable();

                static uint32_t startupTimer = 0;
                static bool enteredStarting = false;
                static int initialDesiredSpeed = 0;
                static bool sliderMoved = false;
                static bool alignDone = false;

                if (!enteredStarting)
                {
                    startupTimer = 0;
                    enteredStarting = true;
                    sliderMoved = false;
                    alignDone = false;
                    initialDesiredSpeed = g_ui32DesiredSpeedRPM;
                    Motor_SetPowerThreshold(150.0f);

                    SafeUARTprintf("=== MOTOR_STARTING: Armed - Waiting for slider movement ===\r\n");
                }

                // === PHASE 1: Wait for user to move the slider ===
                if (!sliderMoved)
                {
                    Motor_SetDutyPercent(0.0f);                    // Motor not spinning yet

                    if (abs(g_ui32DesiredSpeedRPM - initialDesiredSpeed) > 100)   // Slider moved
                    {
                        sliderMoved = true;
                        startupTimer = 0;
                        Motor_StartOpenLoopStartup();
                        SafeUARTprintf("STARTING: Slider moved -> Beginning open-loop startup\r\n");
                    }
                }
                else
                {
                    // === PHASE 2: Do Align + Open-Loop Stepping ===
                    startupTimer += MOTOR_CONTROL_PERIOD_MS;

                    if (!alignDone)
                    {
                        // Short Align phase
                        Motor_SetDutyPercent(22.0f);
                        if (startupTimer >= 400)
                        {
                            alignDone = true;
                            startupTimer = 0;
                        }
                    }
                    else
                    {
                        // Stepping phase
                        if ((startupTimer % 35) == 0)
                        {
                            Motor_ForceCommutationStep();
                        }
                        Motor_SetDutyPercent(26.0f);
                    }

                    // Finish startup sequence
                    if (alignDone && startupTimer >= 2200)
                    {
                        Motor_StopOpenLoopStartup();
                        Motor_SetPowerThreshold(50.0f);
                        enteredStarting = false;
                        sliderMoved = false;
                        alignDone = false;
                        startupTimer = 0;

                        s_fCommandedRPM = 600.0f;
                        // Keep whatever speed the user set on the slider

                        SafeUARTprintf("Motor: Open-loop complete -> RUNNING\r\n");
                        SetMotorState(MOTOR_RUNNING);
                    }
                }
                break;
            }
            case MOTOR_RUNNING:
            {
                if (externalFaultActive)
                {
                    Motor_StopOpenLoopStartup();
                    s_fCommandedRPM = (float)Hfilt_avg10;
                    SetMotorState(MOTOR_ESTOP_BRAKING);
                    break;
                }

                motorStatus = Motor_GetStatus();

                // === AUTO RESTART: If motor stalls (RPM near 0) while we want speed ===
                static uint8_t stallCounter = 0;
                const uint8_t STALL_THRESHOLD = 8;        // ~80ms at 10ms loop
                const float   MIN_SPEED_RPM   = 60.0f;

                if (g_ui32DesiredSpeedRPM > 300 && motorStatus.actual_speed_rpm < MIN_SPEED_RPM)
                {
                    stallCounter++;
                    if (stallCounter >= STALL_THRESHOLD)
                    {
                        stallCounter = 0;
                        SafeUARTprintf("Motor: Stall detected -> restarting startup sequence\r\n");
                        SetMotorState(MOTOR_STARTING);     // Go back to STARTING
                        break;
                    }
                }
                else
                {
                    stallCounter = 0;   // Reset counter if motor is spinning or no demand
                }

                // === Normal ramp control ===
                float desiredRPM = (float)g_ui32DesiredSpeedRPM;
                s_fCommandedRPM = RampToTarget(s_fCommandedRPM, desiredRPM, ACCEL_RATE_RPM_PER_S, DT_SECONDS);

                if (s_fCommandedRPM < 0.0f)    s_fCommandedRPM = 0.0f;
                if (s_fCommandedRPM > 3000.0f) s_fCommandedRPM = 3000.0f;

                float duty = SpeedToDutyPercent((uint32_t)(s_fCommandedRPM + 0.5f));
                Motor_SetDutyPercent(duty);
                Motor_EnableCommutation();
                break;
            }

            case MOTOR_ESTOP_BRAKING:
            {
                // Ramp down at 1000 rpm/s
                s_fCommandedRPM = RampToTarget(s_fCommandedRPM, 0.0f, 
                                            EBRAKE_DECEL_RATE_RPM_PER_S, DT_SECONDS);

                // === CRITICAL FIX: Never allow negative commanded speed ===
                if (s_fCommandedRPM < 0.0f) s_fCommandedRPM = 0.0f;

                float duty = SpeedToDutyPercent((uint32_t)s_fCommandedRPM);   // safe cast
                Motor_SetDutyPercent(duty);

                // Use filtered average for stop detection
                static uint8_t stoppedCount = 0;
                if (Hfilt_avg10 <= 300)
                {
                    stoppedCount++;
                    if (stoppedCount >= 6)
                    {
                        stoppedCount = 0;

                        // Force kill
                        Motor_DisableCommutation();
                        Motor_Brake();
                        Motor_Disable();

                        s_fCommandedRPM = 0.0f;
                        //g_status.actual_speed_rpm = 0.0f;

                        SafeUARTprintf("Motor: E-STOP complete -> FAULT_LATCHED\r\n");
                        SetMotorState(MOTOR_FAULT_LATCHED);
                    }
                }
                else
                {
                    stoppedCount = 0;
                }
                break;
            }

            case MOTOR_FAULT_LATCHED:
            {
                Motor_DisableCommutation();
                Motor_Brake();
                Motor_Disable();
                s_fCommandedRPM = 0.0f;

                if (g_bMotorFaultAckRequested || g_bFaultAcknowledged)
                {
                    g_bMotorFaultAckRequested = false;
                    g_bFaultAcknowledged = false;

                    Motor_ClearFault();
                    ClearAllFaults();

                    SafeUARTprintf("Motor: Fault acknowledged -> IDLE\r\n");
                    SetMotorState(MOTOR_IDLE);
                }
                break;
            }

            default:
                SetMotorState(MOTOR_IDLE);
                break;
        }

        Motor_Update(MOTOR_CONTROL_PERIOD_MS);
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(MOTOR_CONTROL_PERIOD_MS));
    }
}

// Simple speed to duty mapping (replace with PI later if desired)
float SpeedToDutyPercent(uint32_t rpm)
{
    const float MAX_RPM = 3000.0f;
    float duty = ((float)rpm / MAX_RPM) * 70.0f;   // max ~70% duty
    if (duty > 70.0f) duty = 70.0f;
    if (duty < 5.0f && rpm > 0) duty = 5.0f;
    return duty;
}

void SetMotorState(MotorState_t newState)
{
    if (newState == g_eMotorState)
        return;

    g_eMotorState = newState;

    if (xEventGroup)
        xEventGroupSetBits(xEventGroup, EVENT_MOTOR_STATE_CHANGE);

    // Show/Hide START button based on state
    if (g_eMotorState == MOTOR_IDLE)
        ShowStartButton();
    else
        HideStartButton();

    if (g_ui32Panel == 0)
    {
        UpdateMotorStateCanvas();
        UpdateFaultPanel();
    }
}

float RampToTarget(float current, float target, float maxRatePerSecond, float dtSeconds)
{
    float error = target - current;
    float maxDelta = maxRatePerSecond * dtSeconds;

    if (error > maxDelta) return current + maxDelta;
    if (error < -maxDelta) return current - maxDelta;
    return target;
}


float SpeedController_Update(uint32_t targetRpm, float actualRpm, 
                             uint32_t Kp_scaled, uint32_t Ki_scaled, bool reset)
{
    static float integral = 0.0f;
    static float lastDuty = 0.0f;

    if (reset)
    {
        integral = 0.0f;
        lastDuty = 0.0f;
    }

    // Feedforward (base duty)
    float ffDuty = 0.0f;
    if (targetRpm > 50)
    {
        ffDuty = ((float)targetRpm / 3000.0f) * 48.0f;
        if (ffDuty < 8.0f) ffDuty = 8.0f;
    }

    // Use **filtered** speed for feedback (much more stable)
    float filteredRpm = (float)Hfilt_avg10;   // ← important change

    float error = (float)targetRpm - filteredRpm;
    integral += error * (MOTOR_CONTROL_PERIOD_MS / 1000.0f);

    // Stronger anti-windup
    if (integral >  30.0f) integral =  30.0f;
    if (integral < -20.0f) integral = -20.0f;

    float Kp = Kp_scaled / 100.0f;
    float Ki = Ki_scaled / 100.0f;

    float fbDuty = (Kp * error) + (Ki * integral);

    // Limit feedback authority (prevents big swings)
    if (fbDuty >  15.0f) fbDuty =  15.0f;
    if (fbDuty < -15.0f) fbDuty = -15.0f;

    float duty = ffDuty + fbDuty;

    if (duty > 65.0f) duty = 65.0f;
    if (duty < 0.0f)  duty = 0.0f;

    // Slower rate limit
    const float maxChange = 3.5f;
    if (duty > lastDuty + maxChange) duty = lastDuty + maxChange;
    else if (duty < lastDuty - maxChange) duty = lastDuty - maxChange;

    lastDuty = duty;
    return duty;
}


