/*
 * motorlib.c
 *
 * DRV8323RH 6x PWM BLDC motor control library.
 *
 * Think of this file as the phase-output driver.  motor_control.c decides
 * what the motor should be doing; this file applies that decision to PWM/GPIO
 * hardware using the DRV8323RH 6x PWM truth table.
 */

#include "motorlib.h"

#include <stdbool.h>
#include <stdint.h>

/*
 * TivaWare includes
 */
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"

#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"

/* ============================================================
 * System assumptions
 * ============================================================
 *
 * This value must match the system clock configured in main.c.
 *
 * If your main clock is not 120 MHz, change this value.
 */

#define MOTORLIB_SYSTEM_CLOCK_HZ     120000000UL


/* ============================================================
 * Internal PWM output aliases and motor configuration
 * ============================================================
 */

#define MOTOR_OUT_INHA      0U
#define MOTOR_OUT_INHB      1U
#define MOTOR_OUT_INHC      2U
#define MOTOR_OUT_INLA      3U
#define MOTOR_OUT_INLB      4U
#define MOTOR_OUT_INLC      5U

#define INHA_GPIO_BASE      GPIO_PORTF_BASE
#define INHA_GPIO_PIN       GPIO_PIN_1
#define INHB_GPIO_BASE      GPIO_PORTF_BASE
#define INHB_GPIO_PIN       GPIO_PIN_3
#define INHC_GPIO_BASE      GPIO_PORTL_BASE
#define INHC_GPIO_PIN       GPIO_PIN_4

#define INLA_GPIO_BASE      GPIO_PORTF_BASE
#define INLA_GPIO_PIN       GPIO_PIN_2
#define INLB_GPIO_BASE      GPIO_PORTG_BASE
#define INLB_GPIO_PIN       GPIO_PIN_0
#define INLC_GPIO_BASE      GPIO_PORTL_BASE
#define INLC_GPIO_PIN       GPIO_PIN_5

#define PWM_INHA            PWM_OUT_1
#define PWM_INHB            PWM_OUT_3
#define PWM_INLA            PWM_OUT_2
#define PWM_INLB            PWM_OUT_4

#define PWM_ENABLED_BITS    (PWM_OUT_1_BIT | PWM_OUT_2_BIT | PWM_OUT_3_BIT | \
                             PWM_OUT_4_BIT)

#define TIMER_PWM_PERIOD_LOAD   ((uint32_t)g_pwmPeriodTicks - 1U)

/*
 * Compile-time direction option.
 *
 * The default forward table uses this Hall order:
 *      001 -> 101 -> 100 -> 110 -> 010 -> 011
 *
 * Set MOTOR_REVERSE to true in motorlib.h or as a build flag to command
 * the opposite torque direction for the same Hall states.
 */
#ifndef MOTOR_REVERSE
#define MOTOR_REVERSE       false
#endif

/*
 * Compile-time phase map.
 *
 * If the motor phase wiring does not match the A/B/C labels below, swap
 * these output-pair definitions instead of scattering pin changes through
 * the commutation table.
 */
#ifndef MOTOR_PHASE_A_HIGH_OUT
#define MOTOR_PHASE_A_HIGH_OUT   MOTOR_OUT_INHA
#endif
#ifndef MOTOR_PHASE_A_LOW_OUT
#define MOTOR_PHASE_A_LOW_OUT    MOTOR_OUT_INLA
#endif
#ifndef MOTOR_PHASE_B_HIGH_OUT
#define MOTOR_PHASE_B_HIGH_OUT   MOTOR_OUT_INHB
#endif
#ifndef MOTOR_PHASE_B_LOW_OUT
#define MOTOR_PHASE_B_LOW_OUT    MOTOR_OUT_INLB
#endif
#ifndef MOTOR_PHASE_C_HIGH_OUT
#define MOTOR_PHASE_C_HIGH_OUT   MOTOR_OUT_INHC
#endif
#ifndef MOTOR_PHASE_C_LOW_OUT
#define MOTOR_PHASE_C_LOW_OUT    MOTOR_OUT_INLC
#endif

typedef enum
{
    PHASE_FLOAT = 0,
    PHASE_LOW,
    PHASE_HIGH_PWM
} PhaseState_t;


/* ============================================================
 * Internal state
 * ============================================================
 */

static uint16_t g_pwmPeriodTicks = 0;
static uint16_t g_motorDutyTicks = 0;
static bool g_motorInitialised = false;


/* ============================================================
 * Internal function prototypes
 * ============================================================
 */

static void prvMotorGPIOInit(void);
static void prvMotorPWMInit(uint16_t pwm_period_us);
static void prvDelayMs(uint32_t delayMs);

static void prvCoastMotor(void);
static void prvBrakeMotor(void);
static void setPhaseA(PhaseState_t state);
static void setPhaseB(PhaseState_t state);
static void setPhaseC(PhaseState_t state);
static void prvSetPhaseOutputs(uint8_t highOutput, uint8_t lowOutput, PhaseState_t state);
static void prvSetOutputLow(uint8_t motorOut);
static void prvSetOutputHigh(uint8_t motorOut);
static void prvSetOutputPWM(uint8_t motorOut, uint16_t pulseTicks);
static void prvSetPWM(uint8_t motorOut, uint16_t pulseTicks);
static uint16_t prvClampDuty(uint16_t duty);
static void prvSetTimerPulse(uint32_t timer, uint16_t pulseTicks);

static void commutate6Step(uint8_t hallState);


/* ============================================================
 * Public functions
 * ============================================================
 */

bool initMotorLib(uint16_t pwm_period_us)
{
    if (pwm_period_us < MOTOR_PWM_FREQ_MIN_US)
    {
        pwm_period_us = MOTOR_PWM_FREQ_MIN_US;
    }

    if (pwm_period_us > MOTOR_PWM_FREQ_MAX_US)
    {
        pwm_period_us = MOTOR_PWM_FREQ_MAX_US;
    }

    g_motorInitialised = false;
    g_pwmPeriodTicks = 0;
    g_motorDutyTicks = 0;

    prvMotorGPIOInit();
    prvMotorPWMInit(pwm_period_us);

    /*
     * Start disabled and coasting.
     */
    prvCoastMotor();
    disableMotor();

    /*
     * Select 6x PWM mode before the power stage is enabled.
     *
     * DRV832x devices do not do Hall six-step commutation internally in
     * 6x mode. Firmware reads Hall feedback, selects a sector, then drives
     * INHA/INLA, INHB/INLB, and INHC/INLC directly.
     */
    setMotorMode(DRV_MODE_6X_PWM);

    /*
     * Give the gate driver time to settle.
     */
    prvDelayMs(10U);

    g_motorInitialised = true;

    return true;
}


void setDuty(uint16_t duty)
{
    if (!g_motorInitialised)
    {
        return;
    }

    g_motorDutyTicks = prvClampDuty(duty);
}


void updateMotor(bool Hall_a, bool Hall_b, bool Hall_c)
{
    uint8_t hallState;

    if (!g_motorInitialised)
    {
        return;
    }

    /*
     * nFAULT is active-low.
     * If a fault exists, immediately coast and disable the driver.
     */
    if (isMotorFaultActive())
    {
        prvCoastMotor();
        disableMotor();
        return;
    }

    /*
     * Build the real Hall sensor state:
     *
     *      bit 2 = Hall A
     *      bit 1 = Hall B
     *      bit 0 = Hall C
     *
     * During open-loop startup, main.c may pass a fake Hall-sector index
     * through this same API. Those bits are not copied to the DRV pins.
     * They only select a firmware commutation sector.
     */
    hallState = 0;

    if (Hall_a)
    {
        hallState |= 0x04;
    }

    if (Hall_b)
    {
        hallState |= 0x02;
    }

    if (Hall_c)
    {
        hallState |= 0x01;
    }

    commutate6Step(hallState);
}


void stopMotor(bool brakeType)
{
    if (!g_motorInitialised)
    {
        return;
    }

    if (brakeType)
    {
        prvBrakeMotor();
    }
    else
    {
        prvCoastMotor();
    }
}


void coastMotor(void)
{
    if (!g_motorInitialised)
    {
        return;
    }

    prvCoastMotor();
}


void brakeMotor(void)
{
    if (!g_motorInitialised)
    {
        return;
    }

    prvBrakeMotor();
}


void enableMotor(void)
{
    /*
     * DRV8323RH ENABLE is normally active-high.
     */
    GPIOPinWrite(DRV_ENABLE_BASE, DRV_ENABLE_PIN, DRV_ENABLE_PIN);
}


void disableMotor(void)
{
    /*
     * DRV8323RH ENABLE is normally active-high.
     */
    GPIOPinWrite(DRV_ENABLE_BASE, DRV_ENABLE_PIN, 0);
}


uint16_t getMotorPWMPeriod(void)
{
    return g_pwmPeriodTicks;
}


void setMotorMode(uint8_t mode)
{
    if (mode > DRV_MODE_INDEPENDENT)
    {
        mode = DRV_MODE_6X_PWM;
    }

    /*
     * Do not change PWM mode while the MOSFETs are active. Disable the
     * gate driver first, then force all six INHx/INLx inputs low.
     */
    disableMotor();

    if (g_pwmPeriodTicks != 0U)
    {
        prvCoastMotor();
    }

    /*
     * Only one MODE GPIO pin is defined.
     *
     * Therefore only bit 0 of mode can actually be applied.
     */
    if ((mode & 0x01U) != 0U)
    {
        GPIOPinWrite(DRV_MODE_BASE, DRV_MODE_PIN, DRV_MODE_PIN);
    }
    else
    {
        GPIOPinWrite(DRV_MODE_BASE, DRV_MODE_PIN, 0);
    }
}


bool isMotorFaultActive(void)
{
    /*
     * nFAULT is active-low.
     */
    return (GPIOPinRead(DRV_nFAULT_BASE, DRV_nFAULT_PIN) == 0);
}


void clearMotorFault(void)
{
    /*
     * A common method to clear latched faults is to pulse ENABLE low.
     */
    disableMotor();
    prvCoastMotor();

    prvDelayMs(5U);

    enableMotor();

    prvDelayMs(5U);
}


/* ============================================================
 * Internal GPIO initialisation
 * ============================================================
 */

static void prvMotorGPIOInit(void)
{
    /*
     * Enable GPIO ports used by:
     *
     *      ENABLE  -> PC6
     *      nFAULT  -> PC7
     *      MODE    -> PL0
     */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);

    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOC))
    {
    }

    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOL))
    {
    }

    /*
     * ENABLE pin.
     */
    GPIOPinTypeGPIOOutput(DRV_ENABLE_BASE, DRV_ENABLE_PIN);
    GPIOPadConfigSet(
        DRV_ENABLE_BASE,
        DRV_ENABLE_PIN,
        GPIO_STRENGTH_4MA,
        GPIO_PIN_TYPE_STD
    );
    GPIOPinWrite(DRV_ENABLE_BASE, DRV_ENABLE_PIN, 0);

    /*
     * MODE pin.
     */
    GPIOPinTypeGPIOOutput(DRV_MODE_BASE, DRV_MODE_PIN);
    GPIOPadConfigSet(
        DRV_MODE_BASE,
        DRV_MODE_PIN,
        GPIO_STRENGTH_4MA,
        GPIO_PIN_TYPE_STD
    );
    GPIOPinWrite(DRV_MODE_BASE, DRV_MODE_PIN, 0);

    /*
     * nFAULT input with weak pull-up.
     */
    GPIOPinTypeGPIOInput(DRV_nFAULT_BASE, DRV_nFAULT_PIN);
    GPIOPadConfigSet(
        DRV_nFAULT_BASE,
        DRV_nFAULT_PIN,
        GPIO_STRENGTH_2MA,
        GPIO_PIN_TYPE_STD_WPU
    );
}


/* ============================================================
 * Internal PWM initialisation
 * ============================================================
 */

static void prvMotorPWMInit(uint16_t pwm_period_us)
{
    uint32_t pwmPeriodTicks;

    /*
     * Enable PWM0, Timer0, and GPIO ports used by motor inputs.
     */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0))
    {
    }

    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF))
    {
    }

    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOG))
    {
    }

    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOL))
    {
    }

    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0))
    {
    }

    /*
     * Pin mapping:
     *
     *      PF1 -> M0PWM1 -> INHA
     *      PF2 -> M0PWM2 -> INLA
     *      PF3 -> M0PWM3 -> INHB
     *      PG0 -> M0PWM4 -> INLB
     *      PL4 -> T0CCP0 -> INHC
     *      PL5 -> T0CCP1 -> INLC
     *
     * Confirm this against your board schematic.
     */
    GPIOPinConfigure(GPIO_PF1_M0PWM1);
    GPIOPinConfigure(GPIO_PF2_M0PWM2);
    GPIOPinConfigure(GPIO_PF3_M0PWM3);
    GPIOPinConfigure(GPIO_PG0_M0PWM4);
    GPIOPinConfigure(GPIO_PL4_T0CCP0);
    GPIOPinConfigure(GPIO_PL5_T0CCP1);

    GPIOPinTypePWM(
        GPIO_PORTF_BASE,
        GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3
    );

    GPIOPinTypePWM(
        GPIO_PORTG_BASE,
        GPIO_PIN_0
    );

    GPIOPinTypeTimer(
        GPIO_PORTL_BASE,
        GPIO_PIN_4 | GPIO_PIN_5
    );

    /*
     * PWM clock = system clock / 1.
     */
    PWMClockSet(PWM0_BASE, PWM_SYSCLK_DIV_1);

    /*
     * Convert requested period in microseconds to PWM timer ticks.
     *
     * Example:
     *      120 MHz clock, 50 us period:
     *
     *      ticks = 120,000,000 * 50 / 1,000,000
     *            = 6000 ticks
     *
     *      PWM frequency = 20 kHz
     */
    pwmPeriodTicks =
        (MOTORLIB_SYSTEM_CLOCK_HZ / 1000000UL) * (uint32_t)pwm_period_us;

    if (pwmPeriodTicks < 2UL)
    {
        pwmPeriodTicks = 2UL;
    }

    if (pwmPeriodTicks > 65535UL)
    {
        pwmPeriodTicks = 65535UL;
    }

    g_pwmPeriodTicks = (uint16_t)pwmPeriodTicks;

    /*
     * PWM generator mapping:
     *
     *      PWM_GEN_0 -> PWM0, PWM1
     *      PWM_GEN_1 -> PWM2, PWM3
     *      PWM_GEN_2 -> PWM4, PWM5
     */
    PWMGenConfigure(
        PWM0_BASE,
        PWM_GEN_0,
        PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC
    );

    PWMGenConfigure(
        PWM0_BASE,
        PWM_GEN_1,
        PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC
    );

    PWMGenConfigure(
        PWM0_BASE,
        PWM_GEN_2,
        PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC
    );

    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, g_pwmPeriodTicks);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, g_pwmPeriodTicks);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, g_pwmPeriodTicks);

    TimerConfigure(
        TIMER0_BASE,
        TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PWM | TIMER_CFG_B_PWM
    );

    /*
     * Keep Timer0 PWM outputs active-high so PL4/PL5 follow the same
     * DRV input convention as the PWM module outputs.
     */
    TimerControlLevel(TIMER0_BASE, TIMER_A, false);
    TimerControlLevel(TIMER0_BASE, TIMER_B, false);

    TimerLoadSet(TIMER0_BASE, TIMER_A, TIMER_PWM_PERIOD_LOAD);
    TimerLoadSet(TIMER0_BASE, TIMER_B, TIMER_PWM_PERIOD_LOAD);

    /*
     * Start with all DRV input pins low: every phase is Hi-Z/coasting.
     */
    prvCoastMotor();

    /*
     * Enable PWM generators.
     */
    PWMGenEnable(PWM0_BASE, PWM_GEN_0);
    PWMGenEnable(PWM0_BASE, PWM_GEN_1);
    PWMGenEnable(PWM0_BASE, PWM_GEN_2);
    TimerEnable(TIMER0_BASE, TIMER_A);
    TimerEnable(TIMER0_BASE, TIMER_B);

    /*
     * Enable the PWM module outputs used by the motor.
     */
    PWMOutputState(
        PWM0_BASE,
        PWM_ENABLED_BITS,
        true
    );
}


static void prvDelayMs(uint32_t delayMs)
{
    while (delayMs-- > 0U)
    {
        SysCtlDelay(MOTORLIB_SYSTEM_CLOCK_HZ / 3000U);
    }
}


/* ============================================================
 * Phase-state and DRV input helpers
 * ============================================================
 *
 * DRV832x 6x PWM per-phase input truth table:
 *
 *      INHx = 0, INLx = 0 -> phase floating / Hi-Z
 *      INHx = 1, INLx = 0 -> phase high/source
 *      INHx = 0, INLx = 1 -> phase low/sink
 *      INHx = 1, INLx = 1 -> phase floating / Hi-Z
 *
 * This firmware intentionally uses 00 for float. 11 is not used as a
 * commanded Hi-Z state.
 */

static void prvCoastMotor(void)
{
    setPhaseA(PHASE_FLOAT);
    setPhaseB(PHASE_FLOAT);
    setPhaseC(PHASE_FLOAT);
}


static void prvBrakeMotor(void)
{
    /*
     * Low-side brake: high-side inputs low, low-side inputs high.
     * This is deliberately separate from coast.
     */
    setPhaseA(PHASE_LOW);
    setPhaseB(PHASE_LOW);
    setPhaseC(PHASE_LOW);
}


static void setPhaseA(PhaseState_t state)
{
    prvSetPhaseOutputs(MOTOR_PHASE_A_HIGH_OUT, MOTOR_PHASE_A_LOW_OUT, state);
}


static void setPhaseB(PhaseState_t state)
{
    prvSetPhaseOutputs(MOTOR_PHASE_B_HIGH_OUT, MOTOR_PHASE_B_LOW_OUT, state);
}


static void setPhaseC(PhaseState_t state)
{
    prvSetPhaseOutputs(MOTOR_PHASE_C_HIGH_OUT, MOTOR_PHASE_C_LOW_OUT, state);
}


static void prvSetPhaseOutputs(uint8_t highOutput, uint8_t lowOutput, PhaseState_t state)
{
    switch (state)
    {
        case PHASE_LOW:
            prvSetOutputLow(highOutput);
            prvSetOutputHigh(lowOutput);
            break;

        case PHASE_HIGH_PWM:
            prvSetOutputLow(lowOutput);
            prvSetOutputPWM(highOutput, g_motorDutyTicks);
            break;

        case PHASE_FLOAT:
        default:
            prvSetOutputLow(highOutput);
            prvSetOutputLow(lowOutput);
            break;
    }
}


static void prvSetOutputLow(uint8_t motorOut)
{
    switch (motorOut)
    {
        case MOTOR_OUT_INHA:
            GPIOPinTypeGPIOOutput(INHA_GPIO_BASE, INHA_GPIO_PIN);
            GPIOPinWrite(INHA_GPIO_BASE, INHA_GPIO_PIN, 0);
            break;

        case MOTOR_OUT_INHB:
            GPIOPinTypeGPIOOutput(INHB_GPIO_BASE, INHB_GPIO_PIN);
            GPIOPinWrite(INHB_GPIO_BASE, INHB_GPIO_PIN, 0);
            break;

        case MOTOR_OUT_INHC:
            GPIOPinTypeGPIOOutput(INHC_GPIO_BASE, INHC_GPIO_PIN);
            GPIOPinWrite(INHC_GPIO_BASE, INHC_GPIO_PIN, 0);
            break;

        case MOTOR_OUT_INLA:
            GPIOPinTypeGPIOOutput(INLA_GPIO_BASE, INLA_GPIO_PIN);
            GPIOPinWrite(INLA_GPIO_BASE, INLA_GPIO_PIN, 0);
            break;

        case MOTOR_OUT_INLB:
            GPIOPinTypeGPIOOutput(INLB_GPIO_BASE, INLB_GPIO_PIN);
            GPIOPinWrite(INLB_GPIO_BASE, INLB_GPIO_PIN, 0);
            break;

        case MOTOR_OUT_INLC:
            GPIOPinTypeGPIOOutput(INLC_GPIO_BASE, INLC_GPIO_PIN);
            GPIOPinWrite(INLC_GPIO_BASE, INLC_GPIO_PIN, 0);
            break;

        default:
            break;
    }
}


static void prvSetOutputHigh(uint8_t motorOut)
{
    switch (motorOut)
    {
        case MOTOR_OUT_INHA:
            GPIOPinTypeGPIOOutput(INHA_GPIO_BASE, INHA_GPIO_PIN);
            GPIOPinWrite(INHA_GPIO_BASE, INHA_GPIO_PIN, INHA_GPIO_PIN);
            break;

        case MOTOR_OUT_INHB:
            GPIOPinTypeGPIOOutput(INHB_GPIO_BASE, INHB_GPIO_PIN);
            GPIOPinWrite(INHB_GPIO_BASE, INHB_GPIO_PIN, INHB_GPIO_PIN);
            break;

        case MOTOR_OUT_INHC:
            GPIOPinTypeGPIOOutput(INHC_GPIO_BASE, INHC_GPIO_PIN);
            GPIOPinWrite(INHC_GPIO_BASE, INHC_GPIO_PIN, INHC_GPIO_PIN);
            break;

        case MOTOR_OUT_INLA:
            GPIOPinTypeGPIOOutput(INLA_GPIO_BASE, INLA_GPIO_PIN);
            GPIOPinWrite(INLA_GPIO_BASE, INLA_GPIO_PIN, INLA_GPIO_PIN);
            break;

        case MOTOR_OUT_INLB:
            GPIOPinTypeGPIOOutput(INLB_GPIO_BASE, INLB_GPIO_PIN);
            GPIOPinWrite(INLB_GPIO_BASE, INLB_GPIO_PIN, INLB_GPIO_PIN);
            break;

        case MOTOR_OUT_INLC:
            GPIOPinTypeGPIOOutput(INLC_GPIO_BASE, INLC_GPIO_PIN);
            GPIOPinWrite(INLC_GPIO_BASE, INLC_GPIO_PIN, INLC_GPIO_PIN);
            break;

        default:
            break;
    }
}


static void prvSetOutputPWM(uint8_t motorOut, uint16_t pulseTicks)
{
    prvSetPWM(motorOut, pulseTicks);

    switch (motorOut)
    {
        case MOTOR_OUT_INHA:
            GPIOPinConfigure(GPIO_PF1_M0PWM1);
            GPIOPinTypePWM(INHA_GPIO_BASE, INHA_GPIO_PIN);
            break;

        case MOTOR_OUT_INHB:
            GPIOPinConfigure(GPIO_PF3_M0PWM3);
            GPIOPinTypePWM(INHB_GPIO_BASE, INHB_GPIO_PIN);
            break;

        case MOTOR_OUT_INHC:
            GPIOPinConfigure(GPIO_PL4_T0CCP0);
            GPIOPinTypeTimer(INHC_GPIO_BASE, INHC_GPIO_PIN);
            break;

        case MOTOR_OUT_INLA:
            GPIOPinConfigure(GPIO_PF2_M0PWM2);
            GPIOPinTypePWM(INLA_GPIO_BASE, INLA_GPIO_PIN);
            break;

        case MOTOR_OUT_INLB:
            GPIOPinConfigure(GPIO_PG0_M0PWM4);
            GPIOPinTypePWM(INLB_GPIO_BASE, INLB_GPIO_PIN);
            break;

        case MOTOR_OUT_INLC:
            GPIOPinConfigure(GPIO_PL5_T0CCP1);
            GPIOPinTypeTimer(INLC_GPIO_BASE, INLC_GPIO_PIN);
            break;

        default:
            break;
    }
}


static void prvSetPWM(uint8_t motorOut, uint16_t pulseTicks)
{
    if (g_pwmPeriodTicks == 0U)
    {
        return;
    }

    if (pulseTicks < 1U)
    {
        pulseTicks = 1U;
    }

    if (pulseTicks >= g_pwmPeriodTicks)
    {
        pulseTicks = g_pwmPeriodTicks - 1U;
    }

    switch (motorOut)
    {
        case MOTOR_OUT_INHA:
            PWMPulseWidthSet(PWM0_BASE, PWM_INHA, pulseTicks);
            break;

        case MOTOR_OUT_INHB:
            PWMPulseWidthSet(PWM0_BASE, PWM_INHB, pulseTicks);
            break;

        case MOTOR_OUT_INHC:
            prvSetTimerPulse(TIMER_A, pulseTicks);
            break;

        case MOTOR_OUT_INLA:
            PWMPulseWidthSet(PWM0_BASE, PWM_INLA, pulseTicks);
            break;

        case MOTOR_OUT_INLB:
            PWMPulseWidthSet(PWM0_BASE, PWM_INLB, pulseTicks);
            break;

        case MOTOR_OUT_INLC:
            prvSetTimerPulse(TIMER_B, pulseTicks);
            break;

        default:
            break;
    }
}


static void prvSetTimerPulse(uint32_t timer, uint16_t pulseTicks)
{
    uint32_t matchValue;

    /*
     * GPTM PWM duty is controlled by the distance between load and match.
     * Keep the same pulse-tick convention as the PWM module outputs.
     */
    matchValue = TIMER_PWM_PERIOD_LOAD - (uint32_t)pulseTicks;

    TimerMatchSet(TIMER0_BASE, timer, matchValue);
}


static uint16_t prvClampDuty(uint16_t duty)
{
    if (g_pwmPeriodTicks == 0U)
    {
        return 0U;
    }

    if (duty >= g_pwmPeriodTicks)
    {
        duty = g_pwmPeriodTicks - 1U;
    }

    /*
     * Avoid commanding 100 percent duty.
     */
    if (duty < 1U)
    {
        duty = 1U;
    }

    return duty;
}


/* ============================================================
 * Hall-sensor six-step commutation
 * ============================================================
 *
 * The Hall state selects a firmware commutation sector. The sector is then
 * translated to commanded phase states, and the phase helpers drive the
 * physical DRV832x input pins.
 *
 * Starting forward table to verify with Hall monitor mode:
 *
 *      001 -> A high/PWM, B low,      C float
 *      101 -> A high/PWM, C low,      B float
 *      100 -> B high/PWM, C low,      A float
 *      110 -> B high/PWM, A low,      C float
 *      010 -> C high/PWM, A low,      B float
 *      011 -> C high/PWM, B low,      A float
 *
 * Real Hall sensors should produce six clean states and should not produce
 * 000 or 111. If the observed order is different, adjust MOTOR_REVERSE,
 * the phase map, or the Hall wiring/bit mapping after scope-testing the
 * INHA/INLA, INHB/INLB, and INHC/INLC inputs.
 */

static void commutate6Step(uint8_t hallState)
{
    /*
     * Avoid overlap between sectors by forcing all phases to 00 first.
     */
    prvCoastMotor();

    switch (hallState)
    {
        case 0x01:
            if (MOTOR_REVERSE)
            {
                setPhaseA(PHASE_LOW);
                setPhaseB(PHASE_HIGH_PWM);
                setPhaseC(PHASE_FLOAT);
            }
            else
            {
                setPhaseA(PHASE_HIGH_PWM);
                setPhaseB(PHASE_LOW);
                setPhaseC(PHASE_FLOAT);
            }
            break;

        case 0x05:
            if (MOTOR_REVERSE)
            {
                setPhaseA(PHASE_LOW);
                setPhaseB(PHASE_FLOAT);
                setPhaseC(PHASE_HIGH_PWM);
            }
            else
            {
                setPhaseA(PHASE_HIGH_PWM);
                setPhaseB(PHASE_FLOAT);
                setPhaseC(PHASE_LOW);
            }
            break;

        case 0x04:
            if (MOTOR_REVERSE)
            {
                setPhaseA(PHASE_FLOAT);
                setPhaseB(PHASE_LOW);
                setPhaseC(PHASE_HIGH_PWM);
            }
            else
            {
                setPhaseA(PHASE_FLOAT);
                setPhaseB(PHASE_HIGH_PWM);
                setPhaseC(PHASE_LOW);
            }
            break;

        case 0x06:
            if (MOTOR_REVERSE)
            {
                setPhaseA(PHASE_HIGH_PWM);
                setPhaseB(PHASE_LOW);
                setPhaseC(PHASE_FLOAT);
            }
            else
            {
                setPhaseA(PHASE_LOW);
                setPhaseB(PHASE_HIGH_PWM);
                setPhaseC(PHASE_FLOAT);
            }
            break;

        case 0x02:
            if (MOTOR_REVERSE)
            {
                setPhaseA(PHASE_HIGH_PWM);
                setPhaseB(PHASE_FLOAT);
                setPhaseC(PHASE_LOW);
            }
            else
            {
                setPhaseA(PHASE_LOW);
                setPhaseB(PHASE_FLOAT);
                setPhaseC(PHASE_HIGH_PWM);
            }
            break;

        case 0x03:
            if (MOTOR_REVERSE)
            {
                setPhaseA(PHASE_FLOAT);
                setPhaseB(PHASE_HIGH_PWM);
                setPhaseC(PHASE_LOW);
            }
            else
            {
                setPhaseA(PHASE_FLOAT);
                setPhaseB(PHASE_LOW);
                setPhaseC(PHASE_HIGH_PWM);
            }
            break;

        case 0x00:
        case 0x07:
        default:
            /*
             * Invalid Hall state: coast immediately. Do not keep driving a
             * stale sector.
             */
            prvCoastMotor();
            break;
    }
}
