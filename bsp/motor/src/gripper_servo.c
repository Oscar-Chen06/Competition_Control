#include "gripper_servo.h"
#include "tim.h"

static enum { UNINITIALIZED, READY, RUNNING } state;

HAL_StatusTypeDef GripperServo_Init(void)
{
    if ((htim3.Instance != TIM3) || (htim3.State == HAL_TIM_STATE_RESET) ||
        (htim3.Init.Prescaler != 71U) || (htim3.Init.Period != 19999U))
    {
        return HAL_ERROR;
    }

    /* 重复初始化不改变已有输出，也不会重置云台所用的 TIM3。 */
    if (state == UNINITIALIZED)
    {
        state = READY;
    }
    return HAL_OK;
}

HAL_StatusTypeDef GripperServo_SetAngle(uint16_t angle)
{
    uint32_t pulse_us;
    uint32_t previous_pulse;
    HAL_StatusTypeDef status;

    if ((state == UNINITIALIZED) || (angle > GRIPPER_SERVO_MAX_ANGLE_DEG))
    {
        return HAL_ERROR;
    }

    /* 线性换算：0°→500 μs，90°→1500 μs，180°→2500 μs。
     * 加半个除数用于四舍五入，减少整数换算误差。
     */
    pulse_us = GRIPPER_SERVO_MIN_PULSE_US +
               ((GRIPPER_SERVO_MAX_PULSE_US - GRIPPER_SERVO_MIN_PULSE_US) *
                (uint32_t)angle + GRIPPER_SERVO_MAX_ANGLE_DEG / 2U) /
               GRIPPER_SERVO_MAX_ANGLE_DEG;
    previous_pulse = __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_2);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pulse_us);

    /* 首次设置角度时启动通道；以后只更新 CCR2，不重复启动。 */
    if (state == READY)
    {
        status = HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
        if (status != HAL_OK)
        {
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, previous_pulse);
            return status;
        }
        state = RUNNING;
    }
    return HAL_OK;
}
