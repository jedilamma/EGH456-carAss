/*
 * motor_control.c
 *
 * Callable BLDC motor backend for commutation, speed control, e-stop handling,
 * and status reporting.  This file does not create tasks or parse UART input;
 * taskman.c calls Motor_ControlStep() from prvMotorTask().
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
 * Motor timing and tuning
 * ============================================================
 */

#define MOTOR_PWM_PERIOD_US                50U
#define MOTOR_SPEED_SAMPLE_PERIOD_MS       10U

#define MOTOR_ALIGN_TIME_MS                800U
#define MOTOR_STARTUP_STEP_COUNT           60U
#define MOTOR_STARTUP_RAMP_START_DELAY_MS  50U
#define MOTOR_STARTUP_RAMP_END_DELAY_MS    8U
#define MOTOR_START_VALID_HALL_EDGES       3U

#define MOTOR_START_DUTY_PERCENT           15.0f
#define MOTOR_MIN_RUNNING_DUTY_PERCENT     5.0f
#define MOTOR_MAX_DUTY_PERCENT             80.0f

#define MOTOR_MAX_SPEED_RPM                3000.0f
#define MOTOR_STOPPED_RPM                  2.0f

#define MOTOR_NORMAL_RAMP_RPM_PER_S        500.0f
#define MOTOR_ESTOP_RAMP_RPM_PER_S         1000.0f

#define MOTOR_PI_KP                        0.020f
#define MOTOR_PI_KI                        0.015f
#define MOTOR_PI_INTEGRAL_MIN              0.0f
#define MOTOR_PI_INTEGRAL_MAX              MOTOR_MAX_DUTY_PERCENT

#define MOTOR_DEFAULT_HALL_EDGES_PER_REV   6.0f

#define MOTOR_SUPPLY_VOLTAGE               24.0f
#define MOTOR_CURRENT_AMP_GAIN             10.0f
#define MOTOR_CURRENT_SHUNT_OHMS           0.007f
#define MOTOR_ADC_VREF                     3.3f
#define MOTOR_ADC_MAX_COUNT                4095.0f
#define MOTOR_POWER_FILTER_ALPHA           0.12f
#define MOTOR_DEFAULT_POWER_THRESHOLD_W    40.0f


#define MOTOR_REVERSE                     0U


static void clearMotorFault(void)
{
}

static bool isMotorFaultActive(void)
{
    return false;
}


uint16_t getMotorPWMPeriod(void)
{
    return (uint16_t)MOTOR_PWM_PERIOD_US;
}

/* ============================================================
 * Hall sensor and six-step sector definitions
 * ============================================================
 *
 * HA -> PM3
 * HB -> PH2
 * HC -> PN2
 */

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

typedef enum
{
    START_PHASE_ALIGN = 0,
    START_PHASE_OPEN_LOOP,
    START_PHASE_VALIDATE_HALL
} MotorStartPhase_t;

/* ============================================================
 * Internal state
 * ============================================================
 */

static MotorStatus_t g_status;
static SemaphoreHandle_t g_statusMutex = NULL;

static volatile uint32_t g_hallEdgeCount = 0U;
static volatile uint8_t g_hallStateCurrent = 0U;
static volatile uint8_t g_hallStatePrevious = 0U;
static volatile uint8_t g_lastValidHallState = HALL_STATE_001;
static volatile bool g_commutationEnabled = false;

static bool g_motorInitialised = false;
static bool g_stopRequested = false;

static float g_hallEdgesPerRev = MOTOR_DEFAULT_HALL_EDGES_PER_REV;
static float g_piIntegral = 0.0f;
static uint32_t g_lastHallEdgeCount = 0U;
static uint32_t g_speedSampleElapsedMs = 0U;

static MotorStartPhase_t g_startPhase = START_PHASE_ALIGN;
static uint32_t g_startPhaseElapsedMs = 0U;
static uint32_t g_startupStep = 0U;
static uint32_t g_startupStepElapsedMs = 0U;
static uint8_t g_startupStepIndex = 0U;
static uint32_t g_startHallEdgeBase = 0U;

/* ============================================================
 * Internal function prototypes
 * ============================================================
 */

static void prvHallInit(void);
static uint8_t prvReadHallState(void);
static bool prvIsValidHallState(uint8_t hallState);
static void prvHallISR(uint32_t gpioBase, uint8_t hallPin);
static void prvCommandCommutationState(uint8_t hallState);

static void prvEnterIdleLocked(void);
static void prvEnterStartingLocked(void);
static void prvEnterRunningLocked(void);
static void prvEnterEStopLocked(MotorControlFault_t reason);
static void prvEnterFaultLatchedLocked(void);

static void prvHandleIdleLocked(float dtSeconds);
static void prvHandleStartingLocked(uint32_t dtMs);
static void prvHandleRunningLocked(float dtSeconds);
static void prvHandleEStopLocked(float dtSeconds);
static void prvHandleFaultLatchedLocked(float dtSeconds);

static void prvUpdateSpeedEstimateLocked(uint32_t elapsedMs);
static void prvRampReferenceLocked(float targetRpm, float rateRpmPerSecond,
                                   float dtSeconds);
static void prvRunPIControllerLocked(float dtSeconds);
static void prvApplyDutyPercentLocked(float dutyPercent);
static void prvApplyBrakeLocked(void);
static void prvCheckPowerTripLocked(void);

static uint32_t prvGetHallEdgeCount(void);
static uint32_t prvGetStartupDelayMs(uint32_t startupStep);
static uint8_t prvGetOpenLoopHallState(uint8_t step);
static uint8_t prvFindHallStepIndex(uint8_t hallState);

static void prvLockStatus(void);
static void prvUnlockStatus(void);

static int32_t prvFixed2ToFixed1(int32_t scaledValue);
static const char *prvFixed1Sign(int32_t scaledValue);
static uint32_t prvFixed1Magnitude(int32_t scaledValue);
static const char *prvStateName(MotorControlState_t state);
static const char *prvFaultName(MotorControlFault_t fault);

static float prvClampFloat(float value, float minValue, float maxValue);
static float prvAbsFloat(float value);

/* ============================================================
 * Public API
 * ============================================================
 */

void Motor_Init(void)
{
    if (g_motorInitialised)
    {
        return;
    }

    g_statusMutex = xSemaphoreCreateMutex();

    g_status.state = MOTOR_CONTROL_STATE_IDLE;
    g_status.fault = MOTOR_CONTROL_FAULT_NONE;
    g_status.desired_speed_rpm = 0.0f;
    g_status.reference_speed_rpm = 0.0f;
    g_status.actual_speed_rpm = 0.0f;
    g_status.pwm_duty_percent = 0.0f;
    g_status.motor_current_a = 0.0f;
    g_status.motor_power_w = 0.0f;
    g_status.power_threshold_w = MOTOR_DEFAULT_POWER_THRESHOLD_W;
    g_status.hall_edge_count = 0U;

    if (!initMotorLib(MOTOR_PWM_PERIOD_US))
    {
        g_status.state = MOTOR_CONTROL_STATE_FAULT_LATCHED;
        g_status.fault = MOTOR_CONTROL_FAULT_POWER;
    }

    prvHallInit();
    stopMotor(false);
    disableMotor();

    g_lastHallEdgeCount = prvGetHallEdgeCount();
    g_motorInitialised = true;
}

bool Motor_IsReady(void)
{
    return g_motorInitialised;
}

void Motor_ControlStep(uint32_t elapsed_ms)
{
    float dtSeconds;

    if (!g_motorInitialised)
    {
        return;
    }

    if (elapsed_ms == 0U)
    {
        elapsed_ms = MOTOR_CONTROL_PERIOD_MS;
    }

    dtSeconds = (float) elapsed_ms / 1000.0f;

    prvLockStatus();

    g_speedSampleElapsedMs += elapsed_ms;
    if (g_speedSampleElapsedMs >= MOTOR_SPEED_SAMPLE_PERIOD_MS)
    {
        prvUpdateSpeedEstimateLocked(g_speedSampleElapsedMs);
        g_speedSampleElapsedMs = 0U;
    }

    prvCheckPowerTripLocked();

    switch (g_status.state)
    {
        case MOTOR_CONTROL_STATE_IDLE:
            prvHandleIdleLocked(dtSeconds);
            break;

        case MOTOR_CONTROL_STATE_STARTING:
            prvHandleStartingLocked(elapsed_ms);
            break;

        case MOTOR_CONTROL_STATE_RUNNING:
            prvHandleRunningLocked(dtSeconds);
            break;

        case MOTOR_CONTROL_STATE_ESTOP_BRAKING:
            prvHandleEStopLocked(dtSeconds);
            break;

        case MOTOR_CONTROL_STATE_FAULT_LATCHED:
        default:
            prvHandleFaultLatchedLocked(dtSeconds);
            break;
    }

    g_status.hall_edge_count = prvGetHallEdgeCount();

    prvUnlockStatus();
}

void Motor_Start(void)
{
    if (!g_motorInitialised)
    {
        return;
    }

    prvLockStatus();

    if ((g_status.state == MOTOR_CONTROL_STATE_IDLE) &&
        (g_status.desired_speed_rpm > MOTOR_STOPPED_RPM))
    {
        prvEnterStartingLocked();
    }

    prvUnlockStatus();
}

void Motor_Stop(void)
{
    if (!g_motorInitialised)
    {
        return;
    }

    prvLockStatus();

    if (g_status.state == MOTOR_CONTROL_STATE_RUNNING)
    {
        g_stopRequested = true;
        g_status.desired_speed_rpm = 0.0f;
    }
    else if (g_status.state == MOTOR_CONTROL_STATE_STARTING)
    {
        prvEnterIdleLocked();
    }
    else if (g_status.state == MOTOR_CONTROL_STATE_IDLE)
    {
        g_status.desired_speed_rpm = 0.0f;
        g_status.reference_speed_rpm = 0.0f;
    }

    prvUnlockStatus();
}

void Motor_SetSpeed(float rpm)
{
    if (!g_motorInitialised)
    {
        return;
    }

    rpm = prvClampFloat(rpm, 0.0f, MOTOR_MAX_SPEED_RPM);

    prvLockStatus();

    if ((g_status.state != MOTOR_CONTROL_STATE_ESTOP_BRAKING) &&
        (g_status.state != MOTOR_CONTROL_STATE_FAULT_LATCHED))
    {
        g_status.desired_speed_rpm = rpm;

        if (rpm <= MOTOR_STOPPED_RPM)
        {
            if (g_status.state == MOTOR_CONTROL_STATE_STARTING)
            {
                prvEnterIdleLocked();
            }
            else if (g_status.state == MOTOR_CONTROL_STATE_RUNNING)
            {
                g_stopRequested = true;
            }
        }
        else
        {
            g_stopRequested = false;
        }
    }

    prvUnlockStatus();
}

void Motor_EStop(MotorControlFault_t reason)
{
    if (!g_motorInitialised)
    {
        return;
    }

    if (reason == MOTOR_CONTROL_FAULT_NONE)
    {
        reason = MOTOR_CONTROL_FAULT_MANUAL_ESTOP;
    }

    prvLockStatus();

    if (g_status.state != MOTOR_CONTROL_STATE_FAULT_LATCHED)
    {
        prvEnterEStopLocked(reason);
    }

    prvUnlockStatus();
}

void Motor_AcknowledgeFault(void)
{
    if (!g_motorInitialised)
    {
        return;
    }

    prvLockStatus();

    if ((g_status.state == MOTOR_CONTROL_STATE_FAULT_LATCHED) ||
        (g_status.state == MOTOR_CONTROL_STATE_ESTOP_BRAKING))
    {
        clearMotorFault();
        prvEnterIdleLocked();
    }

    prvUnlockStatus();
}

float Motor_GetSpeedRPM(void)
{
    return Motor_GetStatus().actual_speed_rpm;
}

float Motor_GetDesiredSpeedRPM(void)
{
    return Motor_GetStatus().desired_speed_rpm;
}

float Motor_GetReferenceSpeedRPM(void)
{
    return Motor_GetStatus().reference_speed_rpm;
}

float Motor_GetPWMDutyPercent(void)
{
    return Motor_GetStatus().pwm_duty_percent;
}

float Motor_GetPowerW(void)
{
    return Motor_GetStatus().motor_power_w;
}

MotorControlState_t Motor_GetState(void)
{
    return Motor_GetStatus().state;
}

MotorControlFault_t Motor_GetFault(void)
{
    return Motor_GetStatus().fault;
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
    if (!g_motorInitialised)
    {
        return;
    }

    if (threshold_w <= 0.0f)
    {
        return;
    }

    prvLockStatus();
    g_status.power_threshold_w = threshold_w;
    prvUnlockStatus();
}

void Motor_SetHallEdgesPerRev(float hall_edges_per_rev)
{
    if (!g_motorInitialised)
    {
        return;
    }

    if (hall_edges_per_rev < 1.0f)
    {
        return;
    }

    prvLockStatus();
    g_hallEdgesPerRev = hall_edges_per_rev;
    prvUnlockStatus();
}

void Motor_UpdateCurrentSense(float phase_a_current_a, float phase_b_current_a)
{
    float phaseCCurrent;
    float totalCurrent;
    float rawPower;
    bool trip;

    if (!g_motorInitialised)
    {
        return;
    }

    phaseCCurrent = -(phase_a_current_a + phase_b_current_a);
    totalCurrent = prvAbsFloat(phase_a_current_a) +
                   prvAbsFloat(phase_b_current_a) +
                   prvAbsFloat(phaseCCurrent);
    rawPower = MOTOR_SUPPLY_VOLTAGE * totalCurrent;
    trip = false;

    prvLockStatus();

    g_status.motor_current_a = totalCurrent;
    g_status.motor_power_w +=
        MOTOR_POWER_FILTER_ALPHA * (rawPower - g_status.motor_power_w);

    if ((g_status.motor_power_w > g_status.power_threshold_w) &&
        (g_status.state != MOTOR_CONTROL_STATE_ESTOP_BRAKING) &&
        (g_status.state != MOTOR_CONTROL_STATE_FAULT_LATCHED))
    {
        trip = true;
    }

    prvUnlockStatus();

    if (trip)
    {
        Motor_EStop(MOTOR_CONTROL_FAULT_POWER);
    }
}

void Motor_UpdateCurrentSenseADC(uint16_t phase_a_adc, uint16_t phase_b_adc)
{
    float phaseAVoltage;
    float phaseBVoltage;
    float phaseACurrent;
    float phaseBCurrent;

    phaseAVoltage = ((float) phase_a_adc * MOTOR_ADC_VREF) / MOTOR_ADC_MAX_COUNT;
    phaseBVoltage = ((float) phase_b_adc * MOTOR_ADC_VREF) / MOTOR_ADC_MAX_COUNT;

    phaseACurrent =
        phaseAVoltage / (MOTOR_CURRENT_AMP_GAIN * MOTOR_CURRENT_SHUNT_OHMS);
    phaseBCurrent =
        phaseBVoltage / (MOTOR_CURRENT_AMP_GAIN * MOTOR_CURRENT_SHUNT_OHMS);

    Motor_UpdateCurrentSense(phaseACurrent, phaseBCurrent);
}

void Motor_PrintStatusUART(void)
{
    MotorStatus_t status;
    int32_t desiredRpmX10;
    int32_t referenceRpmX10;
    int32_t actualRpmX10;
    int32_t dutyX10;
    int32_t powerX10;
    uint32_t desiredRpmMagnitude;
    uint32_t referenceRpmMagnitude;
    uint32_t actualRpmMagnitude;
    uint32_t dutyMagnitude;
    uint32_t powerMagnitude;

    status = Motor_GetStatus();
    desiredRpmX10 = prvFixed2ToFixed1(FloatToX100(status.desired_speed_rpm));
    referenceRpmX10 = prvFixed2ToFixed1(FloatToX100(status.reference_speed_rpm));
    actualRpmX10 = prvFixed2ToFixed1(FloatToX100(status.actual_speed_rpm));
    dutyX10 = prvFixed2ToFixed1(FloatToX100(status.pwm_duty_percent));
    powerX10 = prvFixed2ToFixed1(FloatToX100(status.motor_power_w));
    desiredRpmMagnitude = prvFixed1Magnitude(desiredRpmX10);
    referenceRpmMagnitude = prvFixed1Magnitude(referenceRpmX10);
    actualRpmMagnitude = prvFixed1Magnitude(actualRpmX10);
    dutyMagnitude = prvFixed1Magnitude(dutyX10);
    powerMagnitude = prvFixed1Magnitude(powerX10);

    SafeUARTprintf(
        "%s%u.%u,%s%u.%u,%s%u.%u,%s%u.%u,%s%u.%u,%u,%u\r\n",
        prvFixed1Sign(desiredRpmX10),
        (unsigned int)(desiredRpmMagnitude / 10U),
        (unsigned int)(desiredRpmMagnitude % 10U),
        prvFixed1Sign(referenceRpmX10),
        (unsigned int)(referenceRpmMagnitude / 10U),
        (unsigned int)(referenceRpmMagnitude % 10U),
        prvFixed1Sign(actualRpmX10),
        (unsigned int)(actualRpmMagnitude / 10U),
        (unsigned int)(actualRpmMagnitude % 10U),
        prvFixed1Sign(dutyX10),
        (unsigned int)(dutyMagnitude / 10U),
        (unsigned int)(dutyMagnitude % 10U),
        prvFixed1Sign(powerX10),
        (unsigned int)(powerMagnitude / 10U),
        (unsigned int)(powerMagnitude % 10U),
        (unsigned int)status.state,
        (unsigned int)status.fault);
}

void Motor_PrintStatusDebugUART(void)
{
    MotorStatus_t status;
    int32_t desiredRpmX10;
    int32_t referenceRpmX10;
    int32_t actualRpmX10;
    int32_t dutyX10;
    int32_t powerX10;
    uint32_t desiredRpmMagnitude;
    uint32_t referenceRpmMagnitude;
    uint32_t actualRpmMagnitude;
    uint32_t dutyMagnitude;
    uint32_t powerMagnitude;

    status = Motor_GetStatus();
    desiredRpmX10 = prvFixed2ToFixed1(FloatToX100(status.desired_speed_rpm));
    referenceRpmX10 = prvFixed2ToFixed1(FloatToX100(status.reference_speed_rpm));
    actualRpmX10 = prvFixed2ToFixed1(FloatToX100(status.actual_speed_rpm));
    dutyX10 = prvFixed2ToFixed1(FloatToX100(status.pwm_duty_percent));
    powerX10 = prvFixed2ToFixed1(FloatToX100(status.motor_power_w));
    desiredRpmMagnitude = prvFixed1Magnitude(desiredRpmX10);
    referenceRpmMagnitude = prvFixed1Magnitude(referenceRpmX10);
    actualRpmMagnitude = prvFixed1Magnitude(actualRpmX10);
    dutyMagnitude = prvFixed1Magnitude(dutyX10);
    powerMagnitude = prvFixed1Magnitude(powerX10);

    SafeUARTprintf(
        "STATE=%s, FAULT=%s, DES=%s%u.%u, REF=%s%u.%u, ACT=%s%u.%u, PWM=%s%u.%u, PWR=%s%u.%u, HALL=%u\r\n",
        prvStateName(status.state),
        prvFaultName(status.fault),
        prvFixed1Sign(desiredRpmX10),
        (unsigned int)(desiredRpmMagnitude / 10U),
        (unsigned int)(desiredRpmMagnitude % 10U),
        prvFixed1Sign(referenceRpmX10),
        (unsigned int)(referenceRpmMagnitude / 10U),
        (unsigned int)(referenceRpmMagnitude % 10U),
        prvFixed1Sign(actualRpmX10),
        (unsigned int)(actualRpmMagnitude / 10U),
        (unsigned int)(actualRpmMagnitude % 10U),
        prvFixed1Sign(dutyX10),
        (unsigned int)(dutyMagnitude / 10U),
        (unsigned int)(dutyMagnitude % 10U),
        prvFixed1Sign(powerX10),
        (unsigned int)(powerMagnitude / 10U),
        (unsigned int)(powerMagnitude % 10U),
        (unsigned int)status.hall_edge_count);
}

/* ============================================================
 * State machine
 * ============================================================
 */

static void prvEnterIdleLocked(void)
{
    g_commutationEnabled = false;
    g_stopRequested = false;
    g_piIntegral = 0.0f;

    g_status.state = MOTOR_CONTROL_STATE_IDLE;
    g_status.fault = MOTOR_CONTROL_FAULT_NONE;
    g_status.reference_speed_rpm = 0.0f;
    g_status.actual_speed_rpm = 0.0f;
    g_status.pwm_duty_percent = 0.0f;

    stopMotor(false);
    disableMotor();
}

static void prvEnterStartingLocked(void)
{
    uint8_t hallState;
    uint8_t step;

    if (g_status.desired_speed_rpm <= MOTOR_STOPPED_RPM)
    {
        prvEnterIdleLocked();
        return;
    }

    g_status.state = MOTOR_CONTROL_STATE_STARTING;
    g_status.fault = MOTOR_CONTROL_FAULT_NONE;
    g_status.reference_speed_rpm = 0.0f;
    g_status.pwm_duty_percent = MOTOR_START_DUTY_PERCENT;
    g_stopRequested = false;
    g_piIntegral = 0.0f;

    clearMotorFault();
    enableMotor();
    prvApplyDutyPercentLocked(MOTOR_START_DUTY_PERCENT);

    hallState = prvReadHallState();
    if (prvIsValidHallState(hallState))
    {
        g_lastValidHallState = hallState;
    }

    step = prvFindHallStepIndex(g_lastValidHallState);
    step++;
    if (step >= 6U)
    {
        step = 0U;
    }

    g_startPhase = START_PHASE_ALIGN;
    g_startPhaseElapsedMs = 0U;
    g_startupStep = 0U;
    g_startupStepElapsedMs = 0U;
    g_startupStepIndex = step;
    g_startHallEdgeBase = prvGetHallEdgeCount();
    g_commutationEnabled = true;

    prvCommandCommutationState(HALL_STATE_001);
}

static void prvEnterRunningLocked(void)
{
    g_status.state = MOTOR_CONTROL_STATE_RUNNING;
    g_status.reference_speed_rpm = 0.0f;
    g_piIntegral = MOTOR_START_DUTY_PERCENT;
    g_commutationEnabled = true;
}

static void prvEnterEStopLocked(MotorControlFault_t reason)
{
    if (reason == MOTOR_CONTROL_FAULT_NONE)
    {
        reason = MOTOR_CONTROL_FAULT_MANUAL_ESTOP;
    }

    g_status.state = MOTOR_CONTROL_STATE_ESTOP_BRAKING;
    g_status.fault = reason;
    g_status.desired_speed_rpm = 0.0f;
    g_stopRequested = false;
    g_commutationEnabled = false;
    g_piIntegral = 0.0f;

    prvApplyBrakeLocked();
}

static void prvEnterFaultLatchedLocked(void)
{
    g_status.state = MOTOR_CONTROL_STATE_FAULT_LATCHED;
    g_status.reference_speed_rpm = 0.0f;
    g_status.desired_speed_rpm = 0.0f;
    g_status.pwm_duty_percent = 0.0f;
    g_commutationEnabled = false;

    stopMotor(false);
    disableMotor();
}

static void prvHandleIdleLocked(float dtSeconds)
{
    (void) dtSeconds;

    g_commutationEnabled = false;
    g_status.reference_speed_rpm = 0.0f;
    g_status.pwm_duty_percent = 0.0f;
    stopMotor(false);
    disableMotor();
}

static void prvHandleStartingLocked(uint32_t dtMs)
{
    uint32_t hallEdgesSinceStart;

    if (g_status.desired_speed_rpm <= MOTOR_STOPPED_RPM)
    {
        prvEnterIdleLocked();
        return;
    }

    if (isMotorFaultActive())
    {
        prvEnterEStopLocked(MOTOR_CONTROL_FAULT_POWER);
        return;
    }

    hallEdgesSinceStart = prvGetHallEdgeCount() - g_startHallEdgeBase;
    if (hallEdgesSinceStart >= MOTOR_START_VALID_HALL_EDGES)
    {
        uint8_t hallState;

        hallState = prvReadHallState();
        if (prvIsValidHallState(hallState))
        {
            g_lastValidHallState = hallState;
            prvCommandCommutationState(hallState);
            prvEnterRunningLocked();
            return;
        }
    }

    switch (g_startPhase)
    {
        case START_PHASE_ALIGN:
            prvApplyDutyPercentLocked(MOTOR_START_DUTY_PERCENT);
            prvCommandCommutationState(HALL_STATE_001);
            g_startPhaseElapsedMs += dtMs;

            if (g_startPhaseElapsedMs >= MOTOR_ALIGN_TIME_MS)
            {
                g_startPhase = START_PHASE_OPEN_LOOP;
                g_startPhaseElapsedMs = 0U;
                g_startupStepElapsedMs = 0U;
            }
            break;

        case START_PHASE_OPEN_LOOP:
        case START_PHASE_VALIDATE_HALL:
        default:
        {
            uint32_t delayMs;

            if (g_startPhase == START_PHASE_OPEN_LOOP)
            {
                delayMs = prvGetStartupDelayMs(g_startupStep);
            }
            else
            {
                delayMs = MOTOR_STARTUP_RAMP_END_DELAY_MS;
            }

            g_startupStepElapsedMs += dtMs;
            if (g_startupStepElapsedMs >= delayMs)
            {
                uint8_t hallState;

                prvApplyDutyPercentLocked(MOTOR_START_DUTY_PERCENT);
                prvCommandCommutationState(
                    prvGetOpenLoopHallState(g_startupStepIndex));

                g_startupStepIndex++;
                if (g_startupStepIndex >= 6U)
                {
                    g_startupStepIndex = 0U;
                }

                if (g_startupStep < MOTOR_STARTUP_STEP_COUNT)
                {
                    g_startupStep++;
                }

                if (g_startupStep >= MOTOR_STARTUP_STEP_COUNT)
                {
                    g_startPhase = START_PHASE_VALIDATE_HALL;
                }

                hallState = prvReadHallState();
                if (prvIsValidHallState(hallState))
                {
                    g_lastValidHallState = hallState;
                    prvCommandCommutationState(hallState);
                }

                g_startupStepElapsedMs = 0U;
            }
            break;
        }
    }
}

static void prvHandleRunningLocked(float dtSeconds)
{
    uint8_t hallState;

    if (isMotorFaultActive())
    {
        prvEnterEStopLocked(MOTOR_CONTROL_FAULT_POWER);
        return;
    }

    if (g_stopRequested)
    {
        g_status.desired_speed_rpm = 0.0f;
    }

    prvRampReferenceLocked(g_status.desired_speed_rpm,
                           MOTOR_NORMAL_RAMP_RPM_PER_S,
                           dtSeconds);

    if ((g_status.reference_speed_rpm <= MOTOR_STOPPED_RPM) &&
        (g_status.desired_speed_rpm <= MOTOR_STOPPED_RPM) &&
        (g_status.actual_speed_rpm <= MOTOR_STOPPED_RPM))
    {
        prvEnterIdleLocked();
        return;
    }

    hallState = prvReadHallState();
    if (prvIsValidHallState(hallState))
    {
        g_lastValidHallState = hallState;
        g_commutationEnabled = true;
        prvRunPIControllerLocked(dtSeconds);
        prvCommandCommutationState(hallState);
    }
    else
    {
        prvApplyDutyPercentLocked(0.0f);
        stopMotor(false);
    }
}

static void prvHandleEStopLocked(float dtSeconds)
{
    prvRampReferenceLocked(0.0f, MOTOR_ESTOP_RAMP_RPM_PER_S, dtSeconds);
    prvApplyBrakeLocked();

    if ((g_status.actual_speed_rpm <= MOTOR_STOPPED_RPM) &&
        (g_status.reference_speed_rpm <= MOTOR_STOPPED_RPM))
    {
        prvEnterFaultLatchedLocked();
    }
}

static void prvHandleFaultLatchedLocked(float dtSeconds)
{
    (void) dtSeconds;

    g_commutationEnabled = false;
    g_status.reference_speed_rpm = 0.0f;
    g_status.desired_speed_rpm = 0.0f;
    g_status.pwm_duty_percent = 0.0f;
    stopMotor(false);
    disableMotor();
}

/* ============================================================
 * Control helpers
 * ============================================================
 */

static void prvUpdateSpeedEstimateLocked(uint32_t elapsedMs)
{
    uint32_t hallEdges;
    uint32_t edgeDelta;
    float sampleRpm;
    float elapsedSeconds;

    hallEdges = prvGetHallEdgeCount();
    edgeDelta = hallEdges - g_lastHallEdgeCount;
    g_lastHallEdgeCount = hallEdges;

    elapsedSeconds = (float) elapsedMs / 1000.0f;
    if ((elapsedSeconds > 0.0f) && (g_hallEdgesPerRev > 0.0f))
    {
        sampleRpm = ((float) edgeDelta * 60.0f) /
                    (g_hallEdgesPerRev * elapsedSeconds);
    }
    else
    {
        sampleRpm = 0.0f;
    }

    g_status.actual_speed_rpm +=
        0.45f * (sampleRpm - g_status.actual_speed_rpm);

    if ((edgeDelta == 0U) && (g_status.actual_speed_rpm < MOTOR_STOPPED_RPM))
    {
        g_status.actual_speed_rpm = 0.0f;
    }
}

static void prvRampReferenceLocked(float targetRpm, float rateRpmPerSecond,
                                   float dtSeconds)
{
    float delta;
    float maxDelta;

    targetRpm = prvClampFloat(targetRpm, 0.0f, MOTOR_MAX_SPEED_RPM);
    delta = targetRpm - g_status.reference_speed_rpm;
    maxDelta = rateRpmPerSecond * dtSeconds;

    if (delta > maxDelta)
    {
        delta = maxDelta;
    }
    else if (delta < -maxDelta)
    {
        delta = -maxDelta;
    }

    g_status.reference_speed_rpm += delta;
}

static void prvRunPIControllerLocked(float dtSeconds)
{
    float error;
    float dutyPercent;

    error = g_status.reference_speed_rpm - g_status.actual_speed_rpm;

    g_piIntegral += MOTOR_PI_KI * error * dtSeconds;
    g_piIntegral = prvClampFloat(g_piIntegral,
                                 MOTOR_PI_INTEGRAL_MIN,
                                 MOTOR_PI_INTEGRAL_MAX);

    dutyPercent = (MOTOR_PI_KP * error) + g_piIntegral;

    if (g_status.reference_speed_rpm > MOTOR_STOPPED_RPM)
    {
        dutyPercent = prvClampFloat(dutyPercent,
                                    MOTOR_MIN_RUNNING_DUTY_PERCENT,
                                    MOTOR_MAX_DUTY_PERCENT);
    }
    else
    {
        dutyPercent = 0.0f;
    }

    prvApplyDutyPercentLocked(dutyPercent);
}

static void prvApplyDutyPercentLocked(float dutyPercent)
{
    uint16_t pwmPeriod;
    uint16_t dutyTicks;

    dutyPercent = prvClampFloat(dutyPercent, 0.0f, MOTOR_MAX_DUTY_PERCENT);
    g_status.pwm_duty_percent = dutyPercent;

    if (dutyPercent <= 0.0f)
    {
        stopMotor(false);
        return;
    }

    pwmPeriod = getMotorPWMPeriod();
    dutyTicks = (uint16_t) (((float) pwmPeriod * dutyPercent) / 100.0f);
    setDuty(dutyTicks);
}

static void prvApplyBrakeLocked(void)
{
    g_status.pwm_duty_percent = 0.0f;
    enableMotor();
    stopMotor(true);
}

static void prvCheckPowerTripLocked(void)
{
    if ((g_status.motor_power_w > g_status.power_threshold_w) &&
        (g_status.state != MOTOR_CONTROL_STATE_ESTOP_BRAKING) &&
        (g_status.state != MOTOR_CONTROL_STATE_FAULT_LATCHED))
    {
        prvEnterEStopLocked(MOTOR_CONTROL_FAULT_POWER);
    }
}

/* ============================================================
 * Hall sensors
 * ============================================================
 */

static void prvHallInit(void)
{
    SysCtlPeripheralEnable(HALL_A_GPIO_PERIPH);
    SysCtlPeripheralEnable(HALL_B_GPIO_PERIPH);
    SysCtlPeripheralEnable(HALL_C_GPIO_PERIPH);

    while (!SysCtlPeripheralReady(HALL_A_GPIO_PERIPH))
    {
    }
    while (!SysCtlPeripheralReady(HALL_B_GPIO_PERIPH))
    {
    }
    while (!SysCtlPeripheralReady(HALL_C_GPIO_PERIPH))
    {
    }

    GPIOPinTypeGPIOInput(HALL_A_GPIO_BASE, HALL_A_PIN);
    GPIOPinTypeGPIOInput(HALL_B_GPIO_BASE, HALL_B_PIN);
    GPIOPinTypeGPIOInput(HALL_C_GPIO_BASE, HALL_C_PIN);

    GPIOPadConfigSet(HALL_A_GPIO_BASE, HALL_A_PIN,
                     GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    GPIOPadConfigSet(HALL_B_GPIO_BASE, HALL_B_PIN,
                     GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    GPIOPadConfigSet(HALL_C_GPIO_BASE, HALL_C_PIN,
                     GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

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
    {
        g_lastValidHallState = g_hallStateCurrent;
    }

    GPIOIntEnable(HALL_A_GPIO_BASE, HALL_A_PIN);
    GPIOIntEnable(HALL_B_GPIO_BASE, HALL_B_PIN);
    GPIOIntEnable(HALL_C_GPIO_BASE, HALL_C_PIN);
}

static uint8_t prvReadHallState(void)
{
    uint8_t hallState;

    hallState = 0U;

    if (GPIOPinRead(HALL_A_GPIO_BASE, HALL_A_PIN) != 0)
    {
        hallState |= HALL_LOGICAL_A_BIT;
    }

    if (GPIOPinRead(HALL_B_GPIO_BASE, HALL_B_PIN) != 0)
    {
        hallState |= HALL_LOGICAL_B_BIT;
    }

    if (GPIOPinRead(HALL_C_GPIO_BASE, HALL_C_PIN) != 0)
    {
        hallState |= HALL_LOGICAL_C_BIT;
    }

    return hallState;
}

static bool prvIsValidHallState(uint8_t hallState)
{
    return ((hallState != 0x00U) && (hallState != 0x07U));
}

static void prvHallISR(uint32_t gpioBase, uint8_t hallPin)
{
    uint32_t status;

    status = GPIOIntStatus(gpioBase, true);
    GPIOIntClear(gpioBase, status);

    if ((status & hallPin) != 0U)
    {
        uint8_t hallState;

        hallState = prvReadHallState();
        g_hallStateCurrent = hallState;

        if (hallState != g_hallStatePrevious)
        {
            g_hallStatePrevious = hallState;

            if (prvIsValidHallState(hallState))
            {
                g_lastValidHallState = hallState;
                g_hallEdgeCount++;

                if (g_commutationEnabled)
                {
                    prvCommandCommutationState(hallState);
                }
            }
            else if (g_commutationEnabled)
            {
                stopMotor(false);
            }
        }
    }
}

void Motor_HallPortHISR(void)
{
    prvHallISR(HALL_B_GPIO_BASE, HALL_B_PIN);
}

void Motor_HallPortMISR(void)
{
    prvHallISR(HALL_A_GPIO_BASE, HALL_A_PIN);
}

void Motor_HallPortNISR(void)
{
    prvHallISR(HALL_C_GPIO_BASE, HALL_C_PIN);
}

static void prvCommandCommutationState(uint8_t hallState)
{
    bool hallA;
    bool hallB;
    bool hallC;

    hallA = ((hallState & HALL_LOGICAL_A_BIT) != 0U);
    hallB = ((hallState & HALL_LOGICAL_B_BIT) != 0U);
    hallC = ((hallState & HALL_LOGICAL_C_BIT) != 0U);

    updateMotor(hallA, hallB, hallC);
}

static uint32_t prvGetHallEdgeCount(void)
{
    return g_hallEdgeCount;
}

static uint32_t prvGetStartupDelayMs(uint32_t startupStep)
{
    uint32_t delaySpan;

    if (MOTOR_STARTUP_STEP_COUNT <= 1U)
    {
        return MOTOR_STARTUP_RAMP_END_DELAY_MS;
    }

    if (MOTOR_STARTUP_RAMP_START_DELAY_MS <= MOTOR_STARTUP_RAMP_END_DELAY_MS)
    {
        return MOTOR_STARTUP_RAMP_END_DELAY_MS;
    }

    if (startupStep >= MOTOR_STARTUP_STEP_COUNT)
    {
        startupStep = MOTOR_STARTUP_STEP_COUNT - 1U;
    }

    delaySpan = MOTOR_STARTUP_RAMP_START_DELAY_MS -
                MOTOR_STARTUP_RAMP_END_DELAY_MS;

    return MOTOR_STARTUP_RAMP_START_DELAY_MS -
        ((delaySpan * startupStep) / (MOTOR_STARTUP_STEP_COUNT - 1U));
}

static uint8_t prvGetOpenLoopHallState(uint8_t step)
{
    static const uint8_t forwardSequence[6] =
    {
        HALL_STATE_001,
        HALL_STATE_101,
        HALL_STATE_100,
        HALL_STATE_110,
        HALL_STATE_010,
        HALL_STATE_011
    };
    static const uint8_t reverseSequence[6] =
    {
        HALL_STATE_011,
        HALL_STATE_010,
        HALL_STATE_110,
        HALL_STATE_100,
        HALL_STATE_101,
        HALL_STATE_001
    };

    step %= 6U;

    if (MOTOR_REVERSE)
    {
        return reverseSequence[step];
    }

    return forwardSequence[step];
}

static uint8_t prvFindHallStepIndex(uint8_t hallState)
{
    uint8_t step;

    for (step = 0U; step < 6U; step++)
    {
        if (prvGetOpenLoopHallState(step) == hallState)
        {
            return step;
        }
    }

    return 0U;
}

/* ============================================================
 * Mutex and formatting helpers
 * ============================================================
 */

static void prvLockStatus(void)
{
    if (g_statusMutex != NULL)
    {
        xSemaphoreTake(g_statusMutex, portMAX_DELAY);
    }
}

static void prvUnlockStatus(void)
{
    if (g_statusMutex != NULL)
    {
        xSemaphoreGive(g_statusMutex);
    }
}

static int32_t prvFixed2ToFixed1(int32_t scaledValue)
{
    if (scaledValue >= 0)
    {
        return (scaledValue + 5) / 10;
    }

    return (scaledValue - 5) / 10;
}

static const char *prvFixed1Sign(int32_t scaledValue)
{
    return (scaledValue < 0) ? "-" : "";
}

static uint32_t prvFixed1Magnitude(int32_t scaledValue)
{
    if (scaledValue < 0)
    {
        return (uint32_t) (-scaledValue);
    }

    return (uint32_t) scaledValue;
}

static const char *prvStateName(MotorControlState_t state)
{
    switch (state)
    {
        case MOTOR_CONTROL_STATE_IDLE:
            return "IDLE";
        case MOTOR_CONTROL_STATE_STARTING:
            return "STARTING";
        case MOTOR_CONTROL_STATE_RUNNING:
            return "RUNNING";
        case MOTOR_CONTROL_STATE_ESTOP_BRAKING:
            return "ESTOP_BRAKING";
        case MOTOR_CONTROL_STATE_FAULT_LATCHED:
            return "FAULT_LATCHED";
        default:
            return "UNKNOWN";
    }
}

static const char *prvFaultName(MotorControlFault_t fault)
{
    switch (fault)
    {
        case MOTOR_CONTROL_FAULT_NONE:
            return "NONE";
        case MOTOR_CONTROL_FAULT_POWER:
            return "POWER";
        case MOTOR_CONTROL_FAULT_ACCELERATION:
            return "ACCELERATION";
        case MOTOR_CONTROL_FAULT_DISTANCE:
            return "DISTANCE";
        case MOTOR_CONTROL_FAULT_MANUAL_ESTOP:
            return "MANUAL_ESTOP";
        default:
            return "UNKNOWN";
    }
}

static float prvClampFloat(float value, float minValue, float maxValue)
{
    if (value < minValue)
    {
        return minValue;
    }

    if (value > maxValue)
    {
        return maxValue;
    }

    return value;
}

static float prvAbsFloat(float value)
{
    if (value < 0.0f)
    {
        return -value;
    }

    return value;
}
