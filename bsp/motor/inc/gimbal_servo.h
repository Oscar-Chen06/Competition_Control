#ifndef GIMBAL_SERVO_H
#define GIMBAL_SERVO_H

#include "stm32f1xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* MG996R endpoints must be calibrated on the actual mechanism. */
#ifndef GIMBAL_SERVO_MIN_PULSE_US
#define GIMBAL_SERVO_MIN_PULSE_US 500U
#endif
#ifndef GIMBAL_SERVO_MAX_PULSE_US
#define GIMBAL_SERVO_MAX_PULSE_US 2500U
#endif

/* Clockwise is viewed facing the output shaft. Set to 0 to reverse it. */
#ifndef GIMBAL_SERVO_CW_DECREASES_PULSE
#define GIMBAL_SERVO_CW_DECREASES_PULSE 1U
#endif

#define GIMBAL_SERVO_HOME_SETTLE_MS 1000U

/* Call after MX_TIM3_Init(): 72 MHz timer clock, PSC=71, ARR=19999.
 * Does not start PWM or command a position. Repeated Init() is harmless.
 * The driver owns CH1; do not stop/reinitialize TIM3 externally during use.
 */
HAL_StatusTypeDef GimbalServo_Init(void);

/* Command the starting endpoint on PA6 / TIM3_CH1 and keep holding it.
 * Returns immediately; allow GIMBAL_SERVO_HOME_SETTLE_MS before RotateCW180().
 * This is a commanded position, not sensor-based homing.
 */
HAL_StatusTypeDef GimbalServo_Home(void);

/* Command the opposite endpoint, a nominal clockwise travel of 180 degrees.
 * Requires Home() and its settling time first. A repeated call returns
 * HAL_ERROR until Home() is called again. HAL_OK means command accepted,
 * not motion complete. PWM remains active to hold the target position.
 */
HAL_StatusTypeDef GimbalServo_RotateCW180(void);

#ifdef __cplusplus
}
#endif

#endif /* GIMBAL_SERVO_H */
