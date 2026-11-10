#ifndef GRIPPER_SERVO_H
#define GRIPPER_SERVO_H

#include "stm32f1xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Calibrate these endpoints before choosing the claw's working angles. */
#ifndef GRIPPER_SERVO_MIN_PULSE_US
#define GRIPPER_SERVO_MIN_PULSE_US 500U
#endif
#ifndef GRIPPER_SERVO_MAX_PULSE_US
#define GRIPPER_SERVO_MAX_PULSE_US 2500U
#endif
#define GRIPPER_SERVO_MAX_ANGLE_DEG 180U

/* Call after MX_TIM3_Init(): 72 MHz timer clock, PSC=71, ARR=19999.
 * Does not start PWM or move the claw. Repeated Init() is harmless.
 * The driver owns CH2; do not stop/reinitialize TIM3 externally during use.
 */
HAL_StatusTypeDef GripperServo_Init(void);

/* PA7 / TIM3_CH2: 0..180 degrees, increasing pulse width.
 * Rejects out-of-range angles or calls before Init() with HAL_ERROR.
 * Returns immediately and keeps PWM running to hold the target position.
 * HAL_OK means command accepted, not motion complete.
 */
HAL_StatusTypeDef GripperServo_SetAngle(uint16_t angle);

#ifdef __cplusplus
}
#endif

#endif /* GRIPPER_SERVO_H */
