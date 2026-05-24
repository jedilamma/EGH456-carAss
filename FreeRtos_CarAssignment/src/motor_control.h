/*
 * motor_control.h
 *
 * Callable BLDC motor-control backend for the EGH456 assessment.
 */

#ifndef MOTOR_CONTROL_H_
#define MOTOR_CONTROL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/*
 * taskman.c should call Motor_ControlStep() at this period from prvMotorTask().
 * The slider value in g_ui32Speed can be passed through Motor_SetSpeed() each
 * loop before this step runs.
 */
#define MOTOR_CONTROL_PERIOD_MS 5U

typedef enum
{
    MOTOR_CONTROL_STATE_IDLE = 0,
    MOTOR_CONTROL_STATE_STARTING,
    MOTOR_CONTROL_STATE_RUNNING,
    MOTOR_CONTROL_STATE_ESTOP_BRAKING,
    MOTOR_CONTROL_STATE_FAULT_LATCHED
} MotorControlState_t;


typedef enum
{
    MOTOR_CONTROL_FAULT_NONE = 0,
    MOTOR_CONTROL_FAULT_POWER,
    MOTOR_CONTROL_FAULT_ACCELERATION,
    MOTOR_CONTROL_FAULT_DISTANCE,
    MOTOR_CONTROL_FAULT_MANUAL_ESTOP
} MotorControlFault_t;

typedef struct
{
    MotorControlState_t state;
    MotorControlFault_t fault;

    float desired_speed_rpm;
    float reference_speed_rpm;
    float actual_speed_rpm;

    float pwm_duty_percent;

    float motor_current_a;
    float motor_power_w;

    float power_threshold_w;
    uint32_t hall_edge_count;
} MotorStatus_t;

void Motor_Init(void);
bool Motor_IsReady(void);

/*
 * Run one motor-control tick.  This replaces the old private motor task, so
 * taskman.c owns when the motor backend is serviced.
 */
void Motor_ControlStep(uint32_t elapsed_ms);

/*
 * Command API.  These are safe to call from widget callbacks or taskman.c.
 * A slider value of zero is the normal stop path.
 */
void Motor_Start(void);
void Motor_Stop(void);
void Motor_SetSpeed(float rpm);
void Motor_EStop(MotorControlFault_t reason);
void Motor_AcknowledgeFault(void);

/*
 * Read-only API for GUI integration and debugging.
 */
float Motor_GetSpeedRPM(void);
float Motor_GetDesiredSpeedRPM(void);
float Motor_GetReferenceSpeedRPM(void);
float Motor_GetPWMDutyPercent(void);
float Motor_GetPowerW(void);

MotorControlState_t Motor_GetState(void);
MotorControlFault_t Motor_GetFault(void);
MotorStatus_t Motor_GetStatus(void);

/*
 * Runtime tuning helpers.
 */
void Motor_SetPowerThreshold(float threshold_w);
void Motor_SetHallEdgesPerRev(float hall_edges_per_rev);

/*
 * Current-sense hooks.
 */
void Motor_UpdateCurrentSense(float phase_a_current_a, float phase_b_current_a);
void Motor_UpdateCurrentSenseADC(uint16_t phase_a_adc, uint16_t phase_b_adc);

/*
 * Optional UART output helpers.  These print status only; UART command parsing
 * has been removed so commands come from widgets/taskman instead.
 */
void Motor_PrintStatusUART(void);
void Motor_PrintStatusDebugUART(void);

/*
 * Hall GPIO interrupt handlers.
 */
void Motor_HallPortHISR(void);
void Motor_HallPortMISR(void);
void Motor_HallPortNISR(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_CONTROL_H_ */
