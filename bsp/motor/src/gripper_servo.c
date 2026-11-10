#include "gripper_servo.h"
#include "tim.h"

#if (GRIPPER_SERVO_MIN_PULSE_US == 0U) || \
    (GRIPPER_SERVO_MIN_PULSE_US >= GRIPPER_SERVO_MAX_PULSE_US) || \
    (GRIPPER_SERVO_MAX_PULSE_US >= 20000U)
#error "Invalid gripper servo pulse endpoints"
#endif

static uint8_t initialized;
static uint8_t pwm_started;

HAL_StatusTypeDef GripperServo_Init(void)
{
    if ((htim3.Instance != TIM3) || (htim3.State == HAL_TIM_STATE_RESET) ||
        (htim3.Init.Prescaler != 71U) || (htim3.Init.Period != 19999U))
    {
        return HAL_ERROR;
    }

    initialized = 1U;
    return HAL_OK;
}

HAL_StatusTypeDef GripperServo_SetAngle(uint16_t angle)
{
    uint32_t pulse_us;
    uint32_t previous_pulse;
    HAL_StatusTypeDef status;

    if ((initialized == 0U) || (angle > GRIPPER_SERVO_MAX_ANGLE_DEG))
    {
        return HAL_ERROR;
    }

    pulse_us = GRIPPER_SERVO_MIN_PULSE_US +
               ((GRIPPER_SERVO_MAX_PULSE_US - GRIPPER_SERVO_MIN_PULSE_US) *
                (uint32_t)angle + GRIPPER_SERVO_MAX_ANGLE_DEG / 2U) /
               GRIPPER_SERVO_MAX_ANGLE_DEG;
    previous_pulse = __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_2);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pulse_us);

    if (pwm_started == 0U)
    {
        status = HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
        if (status != HAL_OK)
        {
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, previous_pulse);
            return status;
        }
        pwm_started = 1U;
    }
    return HAL_OK;
}
