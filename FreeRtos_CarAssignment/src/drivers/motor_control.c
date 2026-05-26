/*
 * motor_control.c
 *
 * Low-level BLDC motor driver backend.
 * Hall-sensored six-step commutation + PWM duty + speed estimation + current/power monitoring.
 *
 * State machine, PI controller, startup sequencing and high-level commands have been removed.
 * This is now a pure driver layer. Higher-level logic lives in taskman.c.
 */

#include "motor_control.h"

#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "semphr.h"

#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"

#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"

#include "motorlib.h"

extern void SafeUARTprintf(const char *pcString, ...);
extern int32_t FloatToX100(float value);

/* ============================================================
 * Motor timing and tuning (kept for driver-level use)
 * ============================================================ */

#define MOTOR_PWM_PERIOD_US                50U
#define MOTOR_SPEED_SAMPLE_PERIOD_MS       10U

#define MOTOR_MAX_DUTY_PERCENT             80.0f
#define MOTOR_STOPPED_RPM                  2.0f

#define MOTOR_DEFAULT_HALL_EDGES_PER_REV   6.0f

#define MOTOR_SUPPLY_VOLTAGE               24.0f
#define MOTOR_CURRENT_AMP_GAIN             10.0f
#define MOTOR_CURRENT_SHUNT_OHMS           0.007f
#define MOTOR_ADC_VREF                     3.3f
#define MOTOR_ADC_MAX_COUNT                4095.0f
#define MOTOR_POWER_FILTER_ALPHA           0.12f
#define MOTOR_DEFAULT_POWER_THRESHOLD_W    40.0f

/* Hall sensor pinout */
#define HALL_A_GPIO_PERIPH                 SYSCTL_PERIPH_GPIOM
#define HALL_A_GPIO_BASE                   GPIO_PORTM_BASE
#define HALL_A_PIN                         GPIO_PIN_3
#define HALL_A_INT                         INT_GPIOM

#define HALL_B_GPIO_PERIPH                 SYSCTL_PERIPH_GPIOH
#define HALL_B_GPIO_BASE                   GPIO_PORTH_BASE
#define HALL_B_PIN                         GPIO_PIN_2
#define HALL_B_INT                         INT_GPIOH

#define HALL_C_GPIO_PERIPH                 SYSCTL_PERIPH_GPION
#define HALL_C_GPIO_BASE                   GPIO_PORTN_BASE
#define HALL_C_PIN                         GPIO_PIN_2
#define HALL_C_INT                         INT_GPION

#define HALL_LOGICAL_A_BIT                 0x04U
#define HALL_LOGICAL_B_BIT                 0x02U
#define HALL_LOGICAL_C_BIT                 0x01U

#define HALL_STATE_001                     0x01U
#define HALL_STATE_101                     0x05U
#define HALL_STATE_100                     0x04U
#define HALL_STATE_110                     0x06U
#define HALL_STATE_010                     0x02U
#define HALL_STATE_011                     0x03U

/* ============================================================
 * Internal state (greatly reduced)
 * ============================================================ */

static MotorStatus_t g_status;
static SemaphoreHandle_t g_statusMutex = NULL;

static volatile uint32_t g_hallEdgeCount = 0U;
static volatile uint8_t g_hallStateCurrent = 0U;
static volatile uint8_t g_hallStatePrevious = 0U;
static volatile uint8_t g_lastValidHallState = HALL_STATE_001;
static volatile bool g_commutationEnabled = true;

static bool g_motorInitialised = false;
static bool g_faultActive = false;

static float g_hallEdgesPerRev = MOTOR_DEFAULT_HALL_EDGES_PER_REV;
static uint32_t g_lastHallEdgeCount = 0U;
static uint32_t g_speedSampleElapsedMs = 0U;

/* === Open-Loop Startup State === */
static bool g_inOpenLoopStartup = false;
static uint8_t g_startupStepIndex = 0;
static uint32_t g_startupElapsedMs = 0;

// 6-step commutation sequence (same order as valid Hall states)
static const uint8_t startupSequence[6] = {
    HALL_STATE_011, HALL_STATE_010, HALL_STATE_110,
    HALL_STATE_100, HALL_STATE_101, HALL_STATE_001
};
/* ============================================================
 * Internal prototypes
 * ============================================================ */

static void prvHallInit(void);
static uint8_t prvReadHallState(void);
static bool prvIsValidHallState(uint8_t hallState);
static void prvHallISR(uint32_t gpioBase, uint8_t hallPin);
static void prvCommandCommutationState(uint8_t hallState);

static void prvLockStatus(void);
static void prvUnlockStatus(void);

static void prvUpdateSpeedEstimate(uint32_t elapsedMs);
static void prvApplyDutyPercent(float dutyPercent);
static void prvApplyBrake(void);
static void prvCheckPowerTrip(void);

static uint32_t prvGetHallEdgeCount(void);

static int32_t prvFixed2ToFixed1(int32_t scaledValue);
static const char *prvFixed1Sign(int32_t scaledValue);
static uint32_t prvFixed1Magnitude(int32_t scaledValue);
static const char *prvFaultName(MotorControlFault_t fault);

static float prvClampFloat(float value, float minValue, float maxValue);
static float prvAbsFloat(float value);

/* ============================================================
 * Public API
 * ============================================================ */

void Motor_Init(void)
{
    if (g_motorInitialised) return;

    g_statusMutex = xSemaphoreCreateMutex();

    g_status.actual_speed_rpm   = 0.0f;
    g_status.pwm_duty_percent   = 0.0f;
    g_status.motor_current_a    = 0.0f;
    g_status.motor_power_w      = 0.0f;
    g_status.power_threshold_w  = MOTOR_DEFAULT_POWER_THRESHOLD_W;
    g_status.hall_edge_count    = 0U;
    g_status.fault              = MOTOR_CONTROL_FAULT_NONE;

    if (!initMotorLib(MOTOR_PWM_PERIOD_US))
    {
        g_faultActive = true;
        g_status.fault = MOTOR_CONTROL_FAULT_POWER;
    }

    prvHallInit();
    stopMotor(false);
    disableMotor();

    g_lastHallEdgeCount = prvGetHallEdgeCount();
    g_commutationEnabled = true;
    g_motorInitialised = true;
}

bool Motor_IsReady(void)
{
    return g_motorInitialised;
}

void Motor_Update(uint32_t elapsed_ms)
{
    if (!g_motorInitialised) return;

    if (elapsed_ms == 0U) elapsed_ms = MOTOR_CONTROL_PERIOD_MS;

    prvLockStatus();

    g_speedSampleElapsedMs += elapsed_ms;

    // Only calculate speed every 10ms for stability
    if (g_speedSampleElapsedMs >= 10U)
    {
        prvUpdateSpeedEstimate(g_speedSampleElapsedMs);
        g_speedSampleElapsedMs = 0U;
    }

    prvCheckPowerTrip();
    g_status.hall_edge_count = prvGetHallEdgeCount();

    prvUnlockStatus();
}

void Motor_Enable(void)
{
    if (!g_motorInitialised) return;
    enableMotor();
}

void Motor_Disable(void)
{
    if (!g_motorInitialised) return;
    disableMotor();
    g_status.pwm_duty_percent = 0.0f;
}

void Motor_SetDutyPercent(float dutyPercent)
{
    if (!g_motorInitialised) return;

    prvLockStatus();
    prvApplyDutyPercent(dutyPercent);
    prvUnlockStatus();
}

void Motor_Brake(void)
{
    if (!g_motorInitialised) return;

    prvLockStatus();
    prvApplyBrake();
    prvUnlockStatus();
}

void Motor_ClearFault(void)
{
    if (!g_motorInitialised) return;

    prvLockStatus();
    g_faultActive = false;
    g_status.fault = MOTOR_CONTROL_FAULT_NONE;
    prvUnlockStatus();
}

bool Motor_IsFaultActive(void)
{
    return g_faultActive;
}

float Motor_GetSpeedRPM(void)
{
    return Motor_GetStatus().actual_speed_rpm;
}

float Motor_GetPWMDutyPercent(void)
{
    return Motor_GetStatus().pwm_duty_percent;
}

float Motor_GetPowerW(void)
{
    return Motor_GetStatus().motor_power_w;
}

float Motor_GetCurrentA(void)
{
    return Motor_GetStatus().motor_current_a;
}

MotorStatus_t Motor_GetStatus(void)
{
    MotorStatus_t snapshot;
    prvLockStatus();
    snapshot = g_status;
    snapshot.hall_edge_count = prvGetHallEdgeCount();
    prvUnlockStatus();
    return snapshot;
}

void Motor_SetPowerThreshold(float threshold_w)
{
    if (!g_motorInitialised || threshold_w <= 0.0f) return;
    prvLockStatus();
    g_status.power_threshold_w = threshold_w;
    prvUnlockStatus();
}

void Motor_SetHallEdgesPerRev(float hall_edges_per_rev)
{
    if (!g_motorInitialised || hall_edges_per_rev < 1.0f) return;
    prvLockStatus();
    g_hallEdgesPerRev = hall_edges_per_rev;
    prvUnlockStatus();
}

void Motor_UpdateCurrentSense(float phase_a_current_a, float phase_b_current_a)
{
    if (!g_motorInitialised) return;

    float phaseCCurrent = -(phase_a_current_a + phase_b_current_a);
    float totalCurrent = prvAbsFloat(phase_a_current_a) + prvAbsFloat(phase_b_current_a) + prvAbsFloat(phaseCCurrent);
    float rawPower = MOTOR_SUPPLY_VOLTAGE * totalCurrent;

    prvLockStatus();
    g_status.motor_current_a = totalCurrent;
    g_status.motor_power_w += MOTOR_POWER_FILTER_ALPHA * (rawPower - g_status.motor_power_w);
    prvUnlockStatus();
}

void Motor_UpdateCurrentSenseADC(uint16_t phase_a_adc, uint16_t phase_b_adc)
{
    float phaseAVoltage = ((float)phase_a_adc * MOTOR_ADC_VREF) / MOTOR_ADC_MAX_COUNT;
    float phaseBVoltage = ((float)phase_b_adc * MOTOR_ADC_VREF) / MOTOR_ADC_MAX_COUNT;

    float phaseACurrent = phaseAVoltage / (MOTOR_CURRENT_AMP_GAIN * MOTOR_CURRENT_SHUNT_OHMS);
    float phaseBCurrent = phaseBVoltage / (MOTOR_CURRENT_AMP_GAIN * MOTOR_CURRENT_SHUNT_OHMS);

    Motor_UpdateCurrentSense(phaseACurrent, phaseBCurrent);
}

void Motor_PrintStatusUART(void)
{
    MotorStatus_t status = Motor_GetStatus();
    int32_t dutyX10   = prvFixed2ToFixed1(FloatToX100(status.pwm_duty_percent));
    int32_t powerX10  = prvFixed2ToFixed1(FloatToX100(status.motor_power_w));
    int32_t actualX10 = prvFixed2ToFixed1(FloatToX100(status.actual_speed_rpm));

    SafeUARTprintf("%s%u.%u,%s%u.%u,%s%u.%u,%u,%u\r\n",
        prvFixed1Sign(actualX10), (unsigned)(prvFixed1Magnitude(actualX10)/10), (unsigned)(prvFixed1Magnitude(actualX10)%10),
        prvFixed1Sign(dutyX10),   (unsigned)(prvFixed1Magnitude(dutyX10)/10),   (unsigned)(prvFixed1Magnitude(dutyX10)%10),
        prvFixed1Sign(powerX10),  (unsigned)(prvFixed1Magnitude(powerX10)/10),  (unsigned)(prvFixed1Magnitude(powerX10)%10),
        (unsigned)status.fault, (unsigned)status.hall_edge_count);
}

void Motor_PrintStatusDebugUART(void)
{
    MotorStatus_t status = Motor_GetStatus();
    SafeUARTprintf("FAULT=%s, ACT=%.1f, PWM=%.1f, PWR=%.1f, HALL=%u\r\n",
        prvFaultName(status.fault),
        (double)status.actual_speed_rpm,
        (double)status.pwm_duty_percent,
        (double)status.motor_power_w,
        (unsigned)status.hall_edge_count);
}

/* Internal helpers */

static void prvUpdateSpeedEstimate(uint32_t elapsedMs)
{
    static uint32_t lastEdgeCount = 0;
    uint32_t currentEdges = prvGetHallEdgeCount();

    int32_t delta = (int32_t)(currentEdges - lastEdgeCount);
    if (delta < 0) delta = 0;
    lastEdgeCount = currentEdges;

    const float sampleTimeSec = 0.01f;

    float rpm = 0.0f;

    // Only calculate if we have real movement and commutation is active
    if (delta > 0 && g_hallEdgesPerRev > 0.0f && g_commutationEnabled)
    {
        rpm = (delta * 60.0f) / (g_hallEdgesPerRev * sampleTimeSec);
    }

    const float SPEED_CALIBRATION = 0.18f;     // ← Start with 0.65 and adjust
    rpm = rpm * SPEED_CALIBRATION;

    // Hard sanity clamp
    if (rpm > 4000.0f) rpm = 4000.0f;
    if (rpm < 0.0f)    rpm = 0.0f;

    // Normal filtering (keep this stable)
    g_status.actual_speed_rpm = (0.80f * g_status.actual_speed_rpm) + (0.20f * rpm);

    // === Only force to zero when we are clearly stopping ===
    // This avoids making normal low-speed running noisy
    if (g_status.pwm_duty_percent < 6.0f && !g_commutationEnabled)
    {
        if (g_status.actual_speed_rpm < 50.0f || delta == 0)
        {
            g_status.actual_speed_rpm = 0.0f;
        }
    }
}

static void prvApplyDutyPercent(float dutyPercent)
{
    dutyPercent = prvClampFloat(dutyPercent, 0.0f, MOTOR_MAX_DUTY_PERCENT);
    g_status.pwm_duty_percent = dutyPercent;

    if (dutyPercent > 0.0f)
    {
        g_status.motor_power_w = dutyPercent * 0.8f;
    }
    else
    {
        g_status.motor_power_w = 0.0f;
        g_status.actual_speed_rpm = 0.0f;     // ← Force clear speed immediately
    }

    if (dutyPercent <= 0.0f)
    {
        stopMotor(false);
        //lastEdgeCount = prvGetHallEdgeCount();   // ← Add this line
        g_status.actual_speed_rpm = 0.0f;
        return;
    }

    uint16_t pwmPeriod = MOTOR_PWM_PERIOD_US;
    uint16_t dutyTicks = (uint16_t)((pwmPeriod * dutyPercent) / 100.0f);
    setDuty(dutyTicks);
}

static void prvApplyBrake(void)
{
    g_status.pwm_duty_percent = 0.0f;
    enableMotor();
    stopMotor(true);
}

static void prvCheckPowerTrip(void)
{
    if (g_status.motor_power_w > g_status.power_threshold_w && !g_faultActive)
    {
        g_faultActive = true;
        g_status.fault = MOTOR_CONTROL_FAULT_POWER;
        prvApplyBrake();
    }
}

/* Hall handling (kept intact) */

static void prvHallInit(void)
{
    SysCtlPeripheralEnable(HALL_A_GPIO_PERIPH);
    SysCtlPeripheralEnable(HALL_B_GPIO_PERIPH);
    SysCtlPeripheralEnable(HALL_C_GPIO_PERIPH);

    while (!SysCtlPeripheralReady(HALL_A_GPIO_PERIPH)) {}
    while (!SysCtlPeripheralReady(HALL_B_GPIO_PERIPH)) {}
    while (!SysCtlPeripheralReady(HALL_C_GPIO_PERIPH)) {}

    GPIOPinTypeGPIOInput(HALL_A_GPIO_BASE, HALL_A_PIN);
    GPIOPinTypeGPIOInput(HALL_B_GPIO_BASE, HALL_B_PIN);
    GPIOPinTypeGPIOInput(HALL_C_GPIO_BASE, HALL_C_PIN);

    GPIOPadConfigSet(HALL_A_GPIO_BASE, HALL_A_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    GPIOPadConfigSet(HALL_B_GPIO_BASE, HALL_B_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    GPIOPadConfigSet(HALL_C_GPIO_BASE, HALL_C_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    GPIOIntDisable(HALL_A_GPIO_BASE, HALL_A_PIN);
    GPIOIntDisable(HALL_B_GPIO_BASE, HALL_B_PIN);
    GPIOIntDisable(HALL_C_GPIO_BASE, HALL_C_PIN);
    GPIOIntClear(HALL_A_GPIO_BASE, HALL_A_PIN);
    GPIOIntClear(HALL_B_GPIO_BASE, HALL_B_PIN);
    GPIOIntClear(HALL_C_GPIO_BASE, HALL_C_PIN);

    GPIOIntTypeSet(HALL_A_GPIO_BASE, HALL_A_PIN, GPIO_BOTH_EDGES);
    GPIOIntTypeSet(HALL_B_GPIO_BASE, HALL_B_PIN, GPIO_BOTH_EDGES);
    GPIOIntTypeSet(HALL_C_GPIO_BASE, HALL_C_PIN, GPIO_BOTH_EDGES);

    GPIOIntRegister(HALL_A_GPIO_BASE, Motor_HallPortMISR);
    GPIOIntRegister(HALL_B_GPIO_BASE, Motor_HallPortHISR);
    GPIOIntRegister(HALL_C_GPIO_BASE, Motor_HallPortNISR);

    IntPrioritySet(HALL_A_INT, configMAX_SYSCALL_INTERRUPT_PRIORITY);
    IntPrioritySet(HALL_B_INT, configMAX_SYSCALL_INTERRUPT_PRIORITY);
    IntPrioritySet(HALL_C_INT, configMAX_SYSCALL_INTERRUPT_PRIORITY);

    IntEnable(HALL_A_INT);
    IntEnable(HALL_B_INT);
    IntEnable(HALL_C_INT);

    g_hallStateCurrent = prvReadHallState();
    g_hallStatePrevious = g_hallStateCurrent;
    if (prvIsValidHallState(g_hallStateCurrent))
        g_lastValidHallState = g_hallStateCurrent;

    GPIOIntEnable(HALL_A_GPIO_BASE, HALL_A_PIN);
    GPIOIntEnable(HALL_B_GPIO_BASE, HALL_B_PIN);
    GPIOIntEnable(HALL_C_GPIO_BASE, HALL_C_PIN);
}

static uint8_t prvReadHallState(void)
{
    uint8_t hallState = 0U;
    if (GPIOPinRead(HALL_A_GPIO_BASE, HALL_A_PIN) != 0) hallState |= HALL_LOGICAL_A_BIT;
    if (GPIOPinRead(HALL_B_GPIO_BASE, HALL_B_PIN) != 0) hallState |= HALL_LOGICAL_B_BIT;
    if (GPIOPinRead(HALL_C_GPIO_BASE, HALL_C_PIN) != 0) hallState |= HALL_LOGICAL_C_BIT;
    return hallState;
}

static bool prvIsValidHallState(uint8_t hallState)
{
    return (hallState != 0x00U && hallState != 0x07U);
}

static void prvHallISR(uint32_t gpioBase, uint8_t hallPin)
{
    uint32_t status = GPIOIntStatus(gpioBase, true);
    GPIOIntClear(gpioBase, status);

    if ((status & hallPin) != 0U)
    {
        uint8_t hallState = prvReadHallState();
        g_hallStateCurrent = hallState;

        if (hallState != g_hallStatePrevious)
        {
            g_hallStatePrevious = hallState;
            if (prvIsValidHallState(hallState))
            {
                g_lastValidHallState = hallState;
                g_hallEdgeCount++;
                if (g_commutationEnabled && g_status.pwm_duty_percent > 5.0f && !g_inOpenLoopStartup)
                    prvCommandCommutationState(hallState);
            }
            else if (g_commutationEnabled)
            {
                stopMotor(false);
            }
        }
    }
}

void Motor_HallPortHISR(void) { prvHallISR(HALL_B_GPIO_BASE, HALL_B_PIN); }
void Motor_HallPortMISR(void) { prvHallISR(HALL_A_GPIO_BASE, HALL_A_PIN); }
void Motor_HallPortNISR(void) { prvHallISR(HALL_C_GPIO_BASE, HALL_C_PIN); }

static void prvCommandCommutationState(uint8_t hallState)
{
    bool hallA = (hallState & HALL_LOGICAL_A_BIT) != 0U;
    bool hallB = (hallState & HALL_LOGICAL_B_BIT) != 0U;
    bool hallC = (hallState & HALL_LOGICAL_C_BIT) != 0U;
    updateMotor(hallA, hallB, hallC);
}

static uint32_t prvGetHallEdgeCount(void)
{
    return g_hallEdgeCount;
}

static void prvLockStatus(void)
{
    if (g_statusMutex) xSemaphoreTake(g_statusMutex, portMAX_DELAY);
}

static void prvUnlockStatus(void)
{
    if (g_statusMutex) xSemaphoreGive(g_statusMutex);
}

static int32_t prvFixed2ToFixed1(int32_t scaledValue)
{
    return (scaledValue >= 0) ? (scaledValue + 5) / 10 : (scaledValue - 5) / 10;
}

static const char *prvFixed1Sign(int32_t scaledValue)
{
    return (scaledValue < 0) ? "-" : "";
}

static uint32_t prvFixed1Magnitude(int32_t scaledValue)
{
    return (scaledValue < 0) ? (uint32_t)(-scaledValue) : (uint32_t)scaledValue;
}

static const char *prvFaultName(MotorControlFault_t fault)
{
    switch (fault)
    {
        case MOTOR_CONTROL_FAULT_NONE: return "NONE";
        case MOTOR_CONTROL_FAULT_POWER: return "POWER";
        case MOTOR_CONTROL_FAULT_MANUAL_ESTOP: return "MANUAL_ESTOP";
        default: return "UNKNOWN";
    }
}

static float prvClampFloat(float value, float minValue, float maxValue)
{
    if (value < minValue) return minValue;
    if (value > maxValue) return maxValue;
    return value;
}

static float prvAbsFloat(float value)
{
    return (value < 0.0f) ? -value : value;
}

uint16_t getMotorPWMPeriod(void)
{
    return (uint16_t)MOTOR_PWM_PERIOD_US;
}

uint8_t Motor_GetCurrentHallState(void)
{
    return g_hallStateCurrent;
}

void Motor_EnableCommutation(void)
{
    g_commutationEnabled = true;
}

void Motor_DisableCommutation(void)
{
    g_commutationEnabled = false;
}

// ==================== OPEN-LOOP STARTUP ====================

void Motor_StartOpenLoopStartup(void)
{
    if (!g_motorInitialised) return;

    g_inOpenLoopStartup = true;
    g_startupStepIndex = 0;
    g_startupElapsedMs = 0;
    g_commutationEnabled = true;

    // Start by forcing first sector
    prvCommandCommutationState(startupSequence[0]);
}

void Motor_StopOpenLoopStartup(void)
{
    g_inOpenLoopStartup = false;
}

bool Motor_IsInOpenLoopStartup(void)
{
    return g_inOpenLoopStartup;
}

void Motor_ForceCommutationStep(void)
{
    if (!g_inOpenLoopStartup || !g_motorInitialised) return;

    g_startupStepIndex = (g_startupStepIndex + 1) % 6;
    uint8_t state = startupSequence[g_startupStepIndex];

    bool hallA = (state & HALL_LOGICAL_A_BIT) != 0U;
    bool hallB = (state & HALL_LOGICAL_B_BIT) != 0U;
    bool hallC = (state & HALL_LOGICAL_C_BIT) != 0U;

    updateMotor(hallA, hallB, hallC);
}