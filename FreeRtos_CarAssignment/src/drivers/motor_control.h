/*
 * motor_control.h
 *
 * Low-level BLDC motor driver backend.
 * Provides Hall-sensored six-step commutation, PWM duty control, speed estimation
 * from Hall edges, current/power monitoring and basic fault flagging (power trip).
 *
 * This module is intentionally driver-level only. It does NOT implement any
 * high-level state machine, PI speed control, startup sequencing, ramping or
 * automatic E-stop logic. Those responsibilities belong in the calling task
 * (e.g. prvMotorTask in taskman.c).
 *
 * taskman.c should call Motor_Update() periodically (every MOTOR_CONTROL_PERIOD_MS)
 * and use Motor_SetDutyPercent() / Motor_Enable() / Motor_Brake() etc. for control.
 */

#ifndef MOTOR_CONTROL_H_
#define MOTOR_CONTROL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/*
 * Recommended period at which Motor_Update() should be called from the motor task.
 */
#define MOTOR_CONTROL_PERIOD_MS 10U

typedef enum 
{
    MOTOR_CONTROL_FAULT_NONE = 0,
    MOTOR_CONTROL_FAULT_POWER,
    MOTOR_CONTROL_FAULT_MANUAL_ESTOP
} MotorControlFault_t;

typedef struct
{
    float actual_speed_rpm;
    float pwm_duty_percent;
    float motor_current_a;
    float motor_power_w;
    float power_threshold_w;
    uint32_t hall_edge_count;
    MotorControlFault_t fault;
} MotorStatus_t;

void Motor_Init(void);
bool Motor_IsReady(void);
uint8_t Motor_GetCurrentHallState(void);

/* Periodic service function - call every MOTOR_CONTROL_PERIOD_MS ms.
   Performs speed estimation and power-trip monitoring. */
void Motor_Update(uint32_t elapsed_ms);

/* ===== Low-level driver control (no state machine) ===== */
void Motor_Enable(void);
void Motor_Disable(void);

/* Set PWM duty cycle directly (0.0 ... 100.0 %). Clamped internally. */
void Motor_SetDutyPercent(float duty_percent);

/* Active brake (shorts motor phases). */
void Motor_Brake(void);

void Motor_ClearFault(void);
bool Motor_IsFaultActive(void);

/* Telemetry */
float Motor_GetSpeedRPM(void);
float Motor_GetPWMDutyPercent(void);
float Motor_GetPowerW(void);
float Motor_GetCurrentA(void);
MotorStatus_t Motor_GetStatus(void);

void Motor_SetPowerThreshold(float threshold_w);
void Motor_SetHallEdgesPerRev(float hall_edges_per_rev);
void Motor_EnableCommutation(void);
void Motor_DisableCommutation(void);
/* Current sense hooks - call when new ADC samples or calculated currents are available */
void Motor_UpdateCurrentSense(float phase_a_current_a, float phase_b_current_a);
void Motor_UpdateCurrentSenseADC(uint16_t phase_a_adc, uint16_t phase_b_adc);

/* Debug / status printing helpers */
void Motor_PrintStatusUART(void);
void Motor_PrintStatusDebugUART(void);

/* Hall sensor interrupt handlers (registered internally during Motor_Init) */
void Motor_HallPortHISR(void);
void Motor_HallPortMISR(void);
void Motor_HallPortNISR(void);

// === Open-Loop Startup Support ===
void Motor_StartOpenLoopStartup(void);
void Motor_StopOpenLoopStartup(void);
void Motor_ForceCommutationStep(void);
bool Motor_IsInOpenLoopStartup(void);

uint8_t Motor_GetCurrentHallState(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_CONTROL_H_ */