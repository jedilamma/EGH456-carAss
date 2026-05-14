#include "taskman.h"

// Hardware drivers for buttons and LEDs
static void prvOptTask(void *pvParameters);
static void prvLEDTasksw1(void *pvParameters);
static void readerTask(void *pvParameters);
static void prvDisplayTask(void *pvParameters);
static void prvMotorTask(void *pvParameters);
static void prvFaultSimulatorTask(void *pvParameters);

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
uint32_t g_ui32CrashThreshold        = 750;   // configurable crash limit
uint32_t g_ui32CurrentDistance       = 180;   // live distance to object (cm)
uint32_t g_ui32MinSafeDistance       = 90;    // configurable safe distance
uint32_t g_ui32LuxMinThreshold       = 30;    // low lux trigger
uint32_t g_ui32LuxMaxThreshold       = 950;   // high lux trigger
uint32_t g_ui32EngineTemp            = 55;    // simulated engine temp for heating/cooling
uint32_t g_ui32HeatingThreshold      = 100;
uint32_t g_ui32CoolingThreshold      = 10;

// Already declared in taskman.h as extern
volatile uint32_t g_ui32TimeStamp = 0;
volatile uint8_t  uiIndex = 0;
volatile uint8_t  uiIndex_2 = 0;
volatile uint32_t g_pui32ButtonPressed = 0;

uint32_t g_plotRawHistory[PLOT_HISTORY_SIZE] = {0};
uint32_t g_plotFiltHistory[PLOT_HISTORY_SIZE] = {0};
uint32_t g_plotIndex = 0;

PlotMode_t g_ePlotMode = PLOT_MODE_FILTERED_ONLY;
volatile MotorState_t g_eMotorState = MOTOR_IDLE;


bool g_bDisplayHold = false;
void vCreateTask(void)
{
    prvConfigureLED();
    prvConfigureButton();
    prvConfigureOPT();
    prvConfigure();   
    prvConfigureHWTimer();                  // Queue + Event Group
    SetTimeFromCompileTime();
    
    xTaskCreate(prvLEDTasksw1,   "LED",    configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, NULL);
    xTaskCreate(prvOptTask,      "OPT",    configMINIMAL_STACK_SIZE*2, NULL, tskIDLE_PRIORITY + 2, NULL);    
    xTaskCreate(prvDisplayTask,  "DISPLAY",  configMINIMAL_STACK_SIZE*20, NULL, tskIDLE_PRIORITY + 3, NULL);
    xTaskCreate(prvFaultSimulatorTask, "FAULTSIM", configMINIMAL_STACK_SIZE*1, NULL, tskIDLE_PRIORITY + 2, NULL);
    xTaskCreate(prvMotorTask,    "MOTOR",  configMINIMAL_STACK_SIZE*4, NULL, tskIDLE_PRIORITY + 2, NULL);
    

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

static void readerTask( void *pvParameters )
{
    struct MsgObj xCurrentMsgObj;
    BaseType_t xRecvStatus;
    EventBits_t uxBits;
    uint32_t messagesWaitingAtStart = 0;
    uint32_t messagesDiscarded = 0;
    uint16_t missingSampleCount = 0;

    ( void ) pvParameters;

    SafeUARTprintf("Reader Task Started - Waiting for Events...\r\n");

    for( ;; )
    {
        /* Wait for any of the important events */
        uxBits = xEventGroupWaitBits(xEventGroup,
                                     (EVENT_SENSOR_MISSED | 
                                      EVENT_QUEUE_FULL | 
                                      EVENT_HIGH_THRESHOLD | 
                                      EVENT_LOW_THRESHOLD | 
                                      EVENT_BTN_TOGGLE_PLOT | 
                                      EVENT_DISPLAY_HOLD |
                                      EVENT_MOTOR_STATE_CHANGE),
                                     pdTRUE,           // Clear the bits after reading
                                     pdFALSE,          // Any single bit is enough
                                     pdMS_TO_TICKS(100)); // Timeout 100ms

        if (uxBits & EVENT_MOTOR_STATE_CHANGE)
        {
            SafeUARTprintf("EVENT: Motor state changed -> %d\r\n", g_eMotorState);
            
            // Example automatic fault linking
            if (g_eMotorState == MOTOR_FAULT)
                g_ui32ActiveFaults |= FAULT_EBRAKE;
            else
                g_ui32ActiveFaults &= ~FAULT_EBRAKE;
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

        // if (uxBits & EVENT_BTN_TOGGLE_PLOT)
        // {
        //     SafeUARTprintf("EVENT: Plot mode toggled -> %s\r\n", 
        //                   (g_ePlotMode == PLOT_MODE_RAW_AND_FILTERED) ? 
        //                   "Raw + Filtered" : "Filtered Only");
        // }

        if (uxBits & EVENT_DISPLAY_HOLD)
        {
            SafeUARTprintf("EVENT: Display Hold mode %s\r\n",
                          g_bDisplayHold ? "ACTIVATED" : "DEACTIVATED");
        }

        /* === Read Latest Message from Queue === */
        messagesWaitingAtStart = uxQueueMessagesWaiting(xQueue);

        xRecvStatus = xQueueReceive(xQueue, &xCurrentMsgObj, 0);

        if (xRecvStatus == pdPASS)
        {
            xCurrentMsgObj = xCurrentMsgObj;  // keep newest
            messagesDiscarded = 0;

            /* Drain queue to keep only the latest message */
            while (uxQueueMessagesWaiting(xQueue) > 0)
            {
                if (xQueueReceive(xQueue, &xCurrentMsgObj, 0) == pdPASS)
                {
                    messagesDiscarded++;
                }
            }

            /* ====================== FILTER BAD DATA ====================== */
            if (xCurrentMsgObj.flag == 1 && xCurrentMsgObj.id == 1U)
            {

                // /* === Process Events === */
                if (uxBits & EVENT_SENSOR_MISSED)
                {
                    //SafeUARTprintf("EVENT-SM: ");
                }
                
                //SafeUARTprintf("OPTICAL Sensor\r\n");
                missingSampleCount++;
                // Do NOT update plot with bad data
            }
            else
            {
                /* === Good Data - Process Normally === */
                SafeUARTprintf("RX xQueue id=0x%02x val=%u avg=%u seq=%u tick=%u flag=%u "
                               "Miss:%u InQue:%u Disc:%u\r\n",
                               (unsigned int)xCurrentMsgObj.id,
                               (unsigned int)xCurrentMsgObj.val,
                               (unsigned int)xCurrentMsgObj.avg,
                               (unsigned int)xCurrentMsgObj.seq,
                               (unsigned int)xCurrentMsgObj.tick,
                               (unsigned int)xCurrentMsgObj.flag,
                               missingSampleCount,
                               messagesWaitingAtStart,
                               messagesDiscarded);

                /* Update Plot Only if NOT in Display Hold Mode */
                if (!g_bDisplayHold)
                {
                    g_plotRawHistory[g_plotIndex]  = xCurrentMsgObj.val;
                    g_plotFiltHistory[g_plotIndex] = xCurrentMsgObj.avg;
                    g_plotIndex = (g_plotIndex + 1) % PLOT_HISTORY_SIZE;

                    if (g_ui32Panel == 2)
                    {
                        WidgetPaint((tWidget *)&g_sCanvas3);
                    }
                }
            }
        }
        else
        {
            missingSampleCount++;
        }
        vTaskDelay(pdMS_TO_TICKS(mainRECEIVER_DELAY_MS));
    }
}


static void prvOptTask(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint32_t tStart, tEnd, duration_us;
    uint32_t totalTime = 0, sampleCount = 0;
    uint32_t avgTime_us = 0;
    uint16_t rawData = 0;
    float convertedLux = 0.0f;
    bool success;

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
        tStart = *(volatile uint32_t *)0xE0001004;
        success = sensorOpt3001Read(&rawData);
        tEnd = *(volatile uint32_t *)0xE0001004;

        duration_us = (tEnd - tStart) / (g_ui32SysClock / 1000000UL);

        if (success)
        {

            totalTime += duration_us;
            sampleCount++;
            avgTime_us = totalTime / sampleCount;

            sensorOpt3001Convert(rawData, &convertedLux);
            
            static uint16_t val[10] = {0};
            static uint8_t  idx = 0;
            static uint32_t sum = 0;

            // Remove old value from sum
            sum -= val[idx];

            // Insert new value
            val[idx] = (uint16_t)convertedLux;
            sum += val[idx];

            // Move index (circular buffer)
            idx = (idx + 1) % 10;

            // Calculate average
            filt_avg10 = sum / 10;

            xMsgObj.id   = 1U;
            xMsgObj.val  = convertedLux;
            xMsgObj.avg  = filt_avg10;
            xMsgObj.tick = sampleCount;
            xMsgObj.flag = 0;
            xMsgObj.seq  = avgTime_us;

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
            xMsgObj.id   = 1U;
            xMsgObj.flag = 1;
            xEventGroupSetBits(xEventGroup, EVENT_SENSOR_MISSED);            
        }

        // Send to queue
        if (xQueueSend(xQueue, &xMsgObj, 0) != pdPASS)
        {
            xEventGroupSetBits(xEventGroup, EVENT_QUEUE_FULL);
        }
        // Signal that new data is ready
        xEventGroupSetBits(xEventGroup, EVENT_BTN_TOGGLE_PLOT);  // reuse or create separate if needed
        
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50));
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

    xTaskCreate(readerTask, "Reader", configMINIMAL_STACK_SIZE * 2, NULL,mainQUEUE_RECEIVE_TASK_PRIORITY, NULL);

}

static void prvConfigureLED( void )
{
    /* Configure initial LED state.  PinoutSet() has already configured
     * LED I/O. */
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
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER0_BASE, TIMER_A, g_ui32SysClock / 5);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    IntEnable(INT_TIMER0A);
    IntMasterEnable();
    TimerEnable(TIMER0_BASE, TIMER_A);
}

void xTimerHandler(void)
{
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    calcTimeCount();

    UpdateTime();
}

static void prvMotorTask(void *pvParameters)
{
    (void)pvParameters;
    
    SafeUARTprintf("=== Motor Task Started ===\r\n");
    
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    for (;;)
    {
        switch (g_eMotorState)
        {
            case MOTOR_IDLE:
                // Simulate start request (e.g. from button or command)
                vTaskDelay(pdMS_TO_TICKS(3000));        // Stay in IDLE for 3 seconds
                SetMotorState(MOTOR_STARTING);
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
                if ((rand() % 5) == 0)                  // ~20% chance
                    SetMotorState(MOTOR_FAULT);
                else
                    SetMotorState(MOTOR_IDLE);          // Back to idle
                break;

            case MOTOR_FAULT:
                SafeUARTprintf("!!! MOTOR FAULT !!!\r\n");
                g_ui32ActiveFaults |= FAULT_EBRAKE | FAULT_OVERCURRENT;
                vTaskDelay(pdMS_TO_TICKS(5000));        // Stay in fault for 5s
                
                // Auto recover for demo
                g_ui32ActiveFaults = 0;
                SetMotorState(MOTOR_IDLE);
                break;

            default:
                SetMotorState(MOTOR_IDLE);
                break;
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100)); // Small tick
    }
}

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
            g_ui32CurrentDistance < g_ui32MinSafeDistance)
        {
            g_ui32ActiveFaults |= FAULT_EBRAKE;
            SetMotorState(MOTOR_FAULT);
            SafeUARTprintf("!!! E-STOP TRIGGERED (Accel or Distance) !!!\r\n");
        }

        // Heating / Cooling logic
        if (g_ui32EngineTemp > g_ui32HeatingThreshold)
            g_bCoolingOn = true;
        else if (g_ui32EngineTemp < g_ui32CoolingThreshold)
            g_bCoolingOn = false;

        // Update Engine Panel live values
        if (g_ui32Panel == 1)
        {
            WidgetPaint((tWidget *)&g_sEngineCanvas);
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(4000));  // Every 4 seconds
    }
}