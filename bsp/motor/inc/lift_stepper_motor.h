#ifndef LIFT_STEPPER_MOTOR_H
#define LIFT_STEPPER_MOTOR_H

#include "stm32f1xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    LIFT_STEPPER_FORWARD = 0,
    LIFT_STEPPER_REVERSE = 1
} LiftStepperDirection;

/* 1.8-degree motor, driver switches set to 1/4 microstepping.
 * 200 full steps * 4 = 800 pulses/revolution, 0.45 degree/pulse.
 */
#define LIFT_STEPPER_PULSES_PER_REV 800U

/* Match these levels to the driver wiring; reverse direction if necessary. */
#ifndef LIFT_STEPPER_FORWARD_LEVEL
#define LIFT_STEPPER_FORWARD_LEVEL GPIO_PIN_SET
#endif
#ifndef LIFT_STEPPER_ENABLE_LEVEL
#define LIFT_STEPPER_ENABLE_LEVEL GPIO_PIN_RESET
#endif

/* Conservative defaults: 7.5 rpm start, 30 rpm cruise, symmetric ramps. */
#ifndef LIFT_STEPPER_START_PPS
#define LIFT_STEPPER_START_PPS 100U
#endif
#ifndef LIFT_STEPPER_CRUISE_PPS
#define LIFT_STEPPER_CRUISE_PPS 400U
#endif
#ifndef LIFT_STEPPER_RAMP_PULSES
#define LIFT_STEPPER_RAMP_PULSES 100U
#endif

#define LIFT_STEPPER_PULSE_HIGH_US 10U
#define LIFT_STEPPER_DIR_SETUP_US 20U
#define LIFT_STEPPER_ENABLE_SETUP_US 10000U

/* Call after SystemClock_Config() and MX_GPIO_Init().
 * Uses PA10 (PUL), PB12 (DIR), PB15 (ENA) as configured in main.h.
 * Initializes with PUL low and driver disabled; generates no steps.
 * Uses the Cortex-M3 DWT cycle counter without resetting it or using TIM3.
 * Keep the CPU clock and DWT counter running unchanged during a move.
 */
HAL_StatusTypeDef LiftStepper_Init(void);

/* Blocking relative move; call from the main loop, not an interrupt.
 * angle_deg must be finite and nonnegative. Direction is a separate argument.
 * Rounds to the nearest 0.45 degree; supports angles greater than 360 degrees.
 * Zero pulses is a no-op. Unrepresentable pulse counts return HAL_ERROR.
 * Starts/ends slowly; short moves use a triangular speed profile.
 * Leaves PUL low and ENA enabled at completion to retain holding torque.
 * HAL_OK means all pulses were sent, not that position was measured.
 * Interrupts remain enabled; interrupt activity can slow down the move.
 *
 * Example, after checking Init() == HAL_OK:
 * LiftStepper_Move(LIFT_STEPPER_FORWARD, 180.0f); // 400 pulses
 * LiftStepper_Move(LIFT_STEPPER_REVERSE, 90.0f);  // 200 pulses
 */
HAL_StatusTypeDef LiftStepper_Move(LiftStepperDirection direction, float angle_deg);

#ifdef __cplusplus
}
#endif

#endif /* LIFT_STEPPER_MOTOR_H */
