#include "lift_stepper_motor.h"
#include "main.h"
#include <math.h>

#if (LIFT_STEPPER_START_PPS == 0U) || \
    (LIFT_STEPPER_CRUISE_PPS < LIFT_STEPPER_START_PPS) || \
    (LIFT_STEPPER_CRUISE_PPS > (1000000U / (2U * LIFT_STEPPER_PULSE_HIGH_US))) || \
    (LIFT_STEPPER_RAMP_PULSES == 0U)
#error "Invalid lift stepper speed profile"
#endif

static uint8_t initialized;
static uint8_t moving;
static uint32_t cycles_per_us;

static void LiftStepper_DelayUs(uint32_t delay_us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t cycles = delay_us * cycles_per_us;

    /* Unsigned subtraction also handles a cycle-counter wraparound. */
    while ((uint32_t)(DWT->CYCCNT - start) < cycles)
    {
        __NOP();
    }
}

static uint32_t LiftStepper_PeriodUs(uint32_t index, uint32_t count)
{
    uint32_t ramp = index;
    uint32_t remaining = count - 1U - index;
    uint32_t frequency;

    if (ramp > remaining)
    {
        ramp = remaining;
    }
    if (ramp > LIFT_STEPPER_RAMP_PULSES)
    {
        ramp = LIFT_STEPPER_RAMP_PULSES;
    }

    frequency = LIFT_STEPPER_START_PPS +
                (uint32_t)(((uint64_t)(LIFT_STEPPER_CRUISE_PPS -
                                      LIFT_STEPPER_START_PPS) * ramp) /
                           LIFT_STEPPER_RAMP_PULSES);
    /* Round up so the pulse rate never exceeds the requested frequency. */
    return (1000000U + frequency - 1U) / frequency;
}

HAL_StatusTypeDef LiftStepper_Init(void)
{
    uint32_t clock_hz = HAL_RCC_GetHCLKFreq();
    uint32_t start;

    if (moving != 0U)
    {
        return HAL_BUSY;
    }
    if ((__get_IPSR() != 0U) || (__get_PRIMASK() != 0U) ||
        (clock_hz < 1000000U) || ((clock_hz % 1000000U) != 0U))
    {
        return HAL_ERROR;
    }
    if (initialized != 0U)
    {
        return HAL_OK;
    }

    cycles_per_us = clock_hz / 1000000U;
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    start = DWT->CYCCNT;
    for (uint32_t i = 0U; i < 32U; ++i)
    {
        __NOP();
    }
    if (DWT->CYCCNT == start)
    {
        return HAL_ERROR;
    }

    HAL_GPIO_WritePin(PUL_GPIO_Port, PUL_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ENA_GPIO_Port, ENA_Pin,
                      (LIFT_STEPPER_ENABLE_LEVEL == GPIO_PIN_SET) ?
                      GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, LIFT_STEPPER_FORWARD_LEVEL);
    initialized = 1U;
    return HAL_OK;
}

HAL_StatusTypeDef LiftStepper_Move(LiftStepperDirection direction, float angle_deg)
{
    double rounded_count;
    uint32_t count;
    GPIO_PinState direction_level;

    if ((initialized == 0U) || (__get_IPSR() != 0U) ||
        (__get_PRIMASK() != 0U) || !isfinite(angle_deg) || (angle_deg < 0.0f) ||
        ((direction != LIFT_STEPPER_FORWARD) && (direction != LIFT_STEPPER_REVERSE)))
    {
        return HAL_ERROR;
    }
    if (moving != 0U)
    {
        return HAL_BUSY;
    }
    if ((HAL_RCC_GetHCLKFreq() != cycles_per_us * 1000000U) ||
        ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0U) ||
        ((CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) == 0U))
    {
        return HAL_ERROR;
    }

    /* Use double for the range check before converting to uint32_t. */
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

    moving = 1U;
    direction_level = LIFT_STEPPER_FORWARD_LEVEL;
    if (direction == LIFT_STEPPER_REVERSE)
    {
        direction_level = (direction_level == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
    }
    HAL_GPIO_WritePin(PUL_GPIO_Port, PUL_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, direction_level);
    HAL_GPIO_WritePin(ENA_GPIO_Port, ENA_Pin, LIFT_STEPPER_ENABLE_LEVEL);
    LiftStepper_DelayUs(LIFT_STEPPER_ENABLE_SETUP_US);
    LiftStepper_DelayUs(LIFT_STEPPER_DIR_SETUP_US);

    for (uint32_t i = 0U; i < count; ++i)
    {
        uint32_t period_us = LiftStepper_PeriodUs(i, count);

        HAL_GPIO_WritePin(PUL_GPIO_Port, PUL_Pin, GPIO_PIN_SET);
        LiftStepper_DelayUs(LIFT_STEPPER_PULSE_HIGH_US);
        HAL_GPIO_WritePin(PUL_GPIO_Port, PUL_Pin, GPIO_PIN_RESET);
        LiftStepper_DelayUs(period_us - LIFT_STEPPER_PULSE_HIGH_US);
    }

    moving = 0U;
    return HAL_OK;
}
