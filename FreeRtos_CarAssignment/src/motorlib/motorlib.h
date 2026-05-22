/*
 * motorlib.h
 *
 * DRV8323RH 6x PWM BLDC motor control library
 *
 * This library is the low-level hardware layer.  It knows how to drive the
 * DRV8323 input pins, set PWM duty ticks, enable/disable the gate driver, and
 * translate Hall states into six-step phase outputs.
 *
 * The higher-level assessment logic lives in motor_control.c.  That file owns
 * the state machine, speed ramps, PI controller, e-stop handling, and UART API.
 *
 * Created on: May 2025
 * Author: EGH456
 */

#ifndef MOTORLIB_H_
#define MOTORLIB_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/*
 * TivaWare includes
 */
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"

/* ============================================================
 * DRV8323RH control pins
 * ============================================================
 *
 * Change these if your schematic uses different pins.
 */

#define DRV_MODE_BASE       GPIO_PORTL_BASE
#define DRV_MODE_PIN        GPIO_PIN_0

#define DRV_ENABLE_BASE     GPIO_PORTC_BASE
#define DRV_ENABLE_PIN      GPIO_PIN_6

#define DRV_nFAULT_BASE     GPIO_PORTC_BASE
#define DRV_nFAULT_PIN      GPIO_PIN_7


/* ============================================================
 * DRV8323RH mode definitions
 * ============================================================
 *
 * Note:
 * This library is intended for 6x PWM mode.
 *
 * Your current hardware definition only uses one MODE GPIO pin.
 * That means only two logic states can actually be selected from
 * software unless your board has additional hardware straps.
 */

#define DRV_MODE_6X_PWM         0
#define DRV_MODE_3X_PWM         1
#define DRV_MODE_1X_PWM         2
#define DRV_MODE_INDEPENDENT    3


/* ============================================================
 * PWM output mapping
 * ============================================================
 *
 * These are the assumed DRV8323RH input connections:
 *
 *      INHA -> PF1 / M0PWM1
 *      INLA -> PF2 / M0PWM2
 *      INHB -> PF3 / M0PWM3
 *      INLB -> PG0 / M0PWM4
 *
 *      INHC -> PL4 / T0CCP0
 *      INLC -> PL5 / T0CCP1
 */

#define MOTOR_PWM_FREQ_MIN_US       10U
#define MOTOR_PWM_FREQ_MAX_US       100U

#define MOTOR_DEFAULT_PWM_PERIOD_US 50U


/* ============================================================
 * Motor configuration options
 * ============================================================
 *
 * Set true to reverse the firmware six-step commutation table.
 *
 * The expected forward Hall sequence is:
 *      001 -> 101 -> 100 -> 110 -> 010 -> 011
 *
 * Verify the actual sequence in Hall monitor mode by rotating the motor
 * by hand before applying real load.
 */

#ifndef MOTOR_REVERSE
#define MOTOR_REVERSE false
#endif


/* ============================================================
 * Public motor library API
 * ============================================================
 */

/*!
 *  @brief  Initialise GPIO, PWM module, and DRV8323RH control pins.
 *
 *  @param  pwm_period_us
 *          PWM period in microseconds.
 *
 *          Valid range:
 *              10  us = 100 kHz
 *              50  us = 20 kHz
 *              100 us = 10 kHz
 *
 *          Recommended starting value:
 *              50 us = 20 kHz
 *
 *  @return true if initialisation completed successfully.
 */
bool initMotorLib(uint16_t pwm_period_us);


/*!
 *  @brief  Sets the PWM duty cycle used by updateMotor().
 *
 *  @pre    initMotorLib() must be called first.
 *
 *  @param  duty
 *          Duty cycle in PWM timer ticks.
 *
 *          Valid values:
 *              0 to getMotorPWMPeriod()
 *
 *          Example:
 *              setDuty(getMotorPWMPeriod() / 4);   // 25 percent duty
 */
void setDuty(uint16_t duty);


/*!
 *  @brief  Commutates motor phases A, B, C based on Hall sensor state.
 *
 *  @pre    initMotorLib() must be called first.
 *  @pre    setDuty() should be called before updateMotor().
 *
 *  @param  Hall_a
 *          Current value of Hall A sensor.
 *
 *  @param  Hall_b
 *          Current value of Hall B sensor.
 *
 *  @param  Hall_c
 *          Current value of Hall C sensor.
 */
void updateMotor(bool Hall_a, bool Hall_b, bool Hall_c);


/*!
 *  @brief  Brakes or coasts the motor.
 *
 *  @pre    initMotorLib() must be called first.
 *
 *  @param  brakeType
 *          true  = brake, low-side switches on, high-side switches off.
 *          false = coast, all DRV INHx/INLx inputs low.
 */
void stopMotor(bool brakeType);


/*!
 *  @brief  Coasts the motor by floating all phases.
 *
 *          A float: INHA = 0, INLA = 0
 *          B float: INHB = 0, INLB = 0
 *          C float: INHC = 0, INLC = 0
 */
void coastMotor(void);


/*!
 *  @brief  Applies a low-side brake.
 *
 *          A low: INHA = 0, INLA = 1
 *          B low: INHB = 0, INLB = 1
 *          C low: INHC = 0, INLC = 1
 */
void brakeMotor(void);


/*!
 *  @brief  Enables the DRV8323RH gate driver.
 *
 *  Note:
 *          DRV8323RH ENABLE is normally active-high.
 */
void enableMotor(void);


/*!
 *  @brief  Disables the DRV8323RH gate driver.
 *
 *  Note:
 *          DRV8323RH ENABLE is normally active-high, so this drives
 *          ENABLE low.
 */
void disableMotor(void);


/*!
 *  @brief  Returns the configured PWM period in timer ticks.
 *
 *  @return PWM period in timer ticks.
 */
uint16_t getMotorPWMPeriod(void);


/* ============================================================
 * Optional helper API
 * ============================================================
 */

void setMotorMode(uint8_t mode);
bool isMotorFaultActive(void);
void clearMotorFault(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTORLIB_H_ */
