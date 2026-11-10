#include "lift_stepper_motor.h"
#include "main.h"
#include <math.h>

/* 三种状态：未初始化、空闲、正在转动。 */
static enum { UNINITIALIZED, READY, MOVING } state;

HAL_StatusTypeDef LiftStepper_Init(void)
{
    if (state == MOVING)
    {
        return HAL_BUSY;
    }
    if ((__get_IPSR() != 0U) || (__get_PRIMASK() != 0U))
    {
        return HAL_ERROR;
    }
    if (state == READY)
    {
        return HAL_OK;
    }

    /* 初始化不运动：脉冲拉低，TB6600 的 ENA 拉高禁用。 */
    HAL_GPIO_WritePin(PUL_GPIO_Port, PUL_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ENA_GPIO_Port, ENA_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, LIFT_STEPPER_FORWARD_LEVEL);
    state = READY;
    return HAL_OK;
}

HAL_StatusTypeDef LiftStepper_Move(LiftStepperDirection direction, float angle_deg)
{
    double rounded_count;
    uint32_t count;
    GPIO_PinState direction_level;

    if ((state == UNINITIALIZED) || (__get_IPSR() != 0U) ||
        (__get_PRIMASK() != 0U) || !isfinite(angle_deg) || (angle_deg < 0.0f) ||
        ((direction != LIFT_STEPPER_FORWARD) && (direction != LIFT_STEPPER_REVERSE)))
    {
        return HAL_ERROR;
    }
    if (state == MOVING)
    {
        return HAL_BUSY;
    }
    /* 脉冲数 = 角度 × 800 / 360；加 0.5 后取整数实现四舍五入。
     * double 用于转换前的上限检查，避免极大角度转换时溢出。
     */
    rounded_count = (double)angle_deg * LIFT_STEPPER_PULSES_PER_REV / 360.0 + 0.5;
    if (rounded_count >= (double)UINT32_MAX + 1.0)
    {
        return HAL_ERROR;
    }
    count = (uint32_t)rounded_count;
    if (count == 0U)
    {
        return HAL_OK;
    }

    state = MOVING;
    /* 先设置方向和使能，等待驱动器稳定后再发送脉冲。 */
    direction_level = (direction == LIFT_STEPPER_FORWARD) ?
                      LIFT_STEPPER_FORWARD_LEVEL :
                      (GPIO_PinState)!LIFT_STEPPER_FORWARD_LEVEL;
    HAL_GPIO_WritePin(PUL_GPIO_Port, PUL_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, direction_level);
    HAL_GPIO_WritePin(ENA_GPIO_Port, ENA_Pin, LIFT_STEPPER_ENABLE_LEVEL);
    HAL_Delay(LIFT_STEPPER_ENABLE_DELAY_MS);

    for (uint32_t i = 0U; i < count; ++i)
    {
        /* 高、低电平各等待固定毫秒数，循环次数就是脉冲数。 */
        HAL_GPIO_WritePin(PUL_GPIO_Port, PUL_Pin, GPIO_PIN_SET);
        HAL_Delay(LIFT_STEPPER_PULSE_DELAY_MS);
        HAL_GPIO_WritePin(PUL_GPIO_Port, PUL_Pin, GPIO_PIN_RESET);
        HAL_Delay(LIFT_STEPPER_PULSE_DELAY_MS);
    }

    /* 不撤销使能，让升降机构继续保持力矩。 */
    state = READY;
    return HAL_OK;
}
