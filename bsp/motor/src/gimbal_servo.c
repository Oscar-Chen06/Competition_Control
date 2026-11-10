#include "gimbal_servo.h"
#include "tim.h"

/* 一个状态即可表示是否初始化、是否启动 PWM，以及上次目标位置。 */
static enum { UNINITIALIZED, READY, HOME, END } state;

static HAL_StatusTypeDef GimbalServo_WritePulse(uint32_t pulse_us)
{
    uint32_t previous_pulse = __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_1);
    HAL_StatusTypeDef status;

    /* CCR1 的数值就是高电平微秒数；只在第一次动作时启动 PWM。 */
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse_us);
    if (state == READY)
    {
        status = HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
        if (status != HAL_OK)
        {
            /* 启动失败则恢复原脉宽，保持原状态，允许重新尝试。 */
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, previous_pulse);
            return status;
        }
    }
    return HAL_OK;
}

HAL_StatusTypeDef GimbalServo_Init(void)
{
    /* 只确认配置，不重新初始化 TIM3，以免干扰夹爪通道。 */
    if ((htim3.Instance != TIM3) || (htim3.State == HAL_TIM_STATE_RESET) ||
        (htim3.Init.Prescaler != 71U) || (htim3.Init.Period != 19999U))
    {
        return HAL_ERROR;
    }

    if (state == UNINITIALIZED)
    {
        state = READY;
    }
    return HAL_OK;
}

HAL_StatusTypeDef GimbalServo_Home(void)
{
    HAL_StatusTypeDef status;

    if (state == UNINITIALIZED)
    {
        return HAL_ERROR;
    }

    /* 顺时针默认对应脉宽减小，因此从最大脉宽所在端开始。 */
    status = GimbalServo_WritePulse(GIMBAL_SERVO_CW_DECREASES_PULSE ?
                                   GIMBAL_SERVO_MAX_PULSE_US : GIMBAL_SERVO_MIN_PULSE_US);
    if (status == HAL_OK)
    {
        state = HOME;
    }
    return status;
}

HAL_StatusTypeDef GimbalServo_RotateCW180(void)
{
    HAL_StatusTypeDef status;

    /* 必须先下发起点，不能在终点重复执行完整 180° 行程。 */
    if (state != HOME)
    {
        return HAL_ERROR;
    }

    status = GimbalServo_WritePulse(GIMBAL_SERVO_CW_DECREASES_PULSE ?
                                   GIMBAL_SERVO_MIN_PULSE_US : GIMBAL_SERVO_MAX_PULSE_US);
    if (status == HAL_OK)
    {
        state = END;
    }
    return status;
}
