#include "gimbal_servo.h"
#include "tim.h"

#if (GIMBAL_SERVO_MIN_PULSE_US == 0U) || \
    (GIMBAL_SERVO_MIN_PULSE_US >= GIMBAL_SERVO_MAX_PULSE_US) || \
    (GIMBAL_SERVO_MAX_PULSE_US >= 20000U)
#error "Invalid gimbal servo pulse endpoints"
#endif

static uint8_t initialized;
static uint8_t pwm_started;
static uint8_t home_commanded;

static HAL_StatusTypeDef GimbalServo_WritePulse(uint32_t pulse_us)
{
    uint32_t previous_pulse = __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_1);
    HAL_StatusTypeDef status;

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse_us);
    if (pwm_started == 0U)
    {
        status = HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
        if (status != HAL_OK)
        {
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, previous_pulse);
            return status;
        }
        pwm_started = 1U;
    }
    return HAL_OK;
}

HAL_StatusTypeDef GimbalServo_Init(void)
{
    /* The shared timer is owned by MX_TIM3_Init(), never reconfigured here. */
    if ((htim3.Instance != TIM3) || (htim3.State == HAL_TIM_STATE_RESET) ||
        (htim3.Init.Prescaler != 71U) || (htim3.Init.Period != 19999U))
    {
        return HAL_ERROR;
    }

    initialized = 1U;
    return HAL_OK;
}

HAL_StatusTypeDef GimbalServo_Home(void)
{
    HAL_StatusTypeDef status;

    if (initialized == 0U)
    {
        return HAL_ERROR;
    }

#if GIMBAL_SERVO_CW_DECREASES_PULSE
    status = GimbalServo_WritePulse(GIMBAL_SERVO_MAX_PULSE_US);
#else
    status = GimbalServo_WritePulse(GIMBAL_SERVO_MIN_PULSE_US);
#endif
    if (status == HAL_OK)
    {
        home_commanded = 1U;
    }
    return status;
}

HAL_StatusTypeDef GimbalServo_RotateCW180(void)
{
    HAL_StatusTypeDef status;

    if ((initialized == 0U) || (home_commanded == 0U))
    {
        return HAL_ERROR;
    }

#if GIMBAL_SERVO_CW_DECREASES_PULSE
    status = GimbalServo_WritePulse(GIMBAL_SERVO_MIN_PULSE_US);
#else
    status = GimbalServo_WritePulse(GIMBAL_SERVO_MAX_PULSE_US);
#endif
    if (status == HAL_OK)
    {
        home_commanded = 0U;
    }
    return status;
}
