#include "lift_stepper_motor.h"

#include <math.h>
#include <stddef.h>

#include "main.h"
#include "tim.h"
#include "tim1_mux.h"

/* TIM1 由 MG513（CH1/PA8）和步进（CH3/PA10）分时复用。 */
static enum { OWNER_NONE, OWNER_MG513, OWNER_STEPPER } owner;

static void tim1_stop_all(void)
{
    (void)HAL_TIM_Base_Stop_IT(&htim1);
    (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    __HAL_TIM_DISABLE_IT(&htim1, TIM_IT_UPDATE);
}

static void tim1_commit(void)
{
    __HAL_TIM_SET_COUNTER(&htim1, 0U);
    htim1.Instance->EGR = TIM_EGR_UG;
    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
}

void Tim1Mux_TakeMg513(uint32_t psc, uint32_t arr)
{
    if (owner == OWNER_MG513)
    {
        return;
    }

    tim1_stop_all();

    htim1.Instance->CR1 &= ~TIM_CR1_ARPE;
    __HAL_TIM_SET_PRESCALER(&htim1, psc);
    __HAL_TIM_SET_AUTORELOAD(&htim1, arr);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0U);
    tim1_commit();

    (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    owner = OWNER_MG513;
}

void Tim1Mux_TakeStepper(uint32_t psc, uint32_t arr)
{
    tim1_stop_all();

    htim1.Instance->CR1 |= TIM_CR1_ARPE;
    __HAL_TIM_SET_PRESCALER(&htim1, psc);
    __HAL_TIM_SET_AUTORELOAD(&htim1, arr);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (arr + 1U) / 2U);
    tim1_commit();
    owner = OWNER_STEPPER;
}

void Tim1Mux_ReleaseStepper(void)
{
    if (owner != OWNER_STEPPER)
    {
        return;
    }

    (void)HAL_TIM_Base_Stop_IT(&htim1);
    (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    __HAL_TIM_DISABLE_IT(&htim1, TIM_IT_UPDATE);
    owner = OWNER_NONE;
}

/* 三种状态：未初始化、空闲、正在转动。 */
static volatile enum { UNINITIALIZED, READY, MOVING } state;
static volatile uint8_t s_abort_requested;
static volatile int32_t s_remaining;
static volatile uint32_t s_freq;
static uint32_t s_f_start;
static uint32_t s_f_max;
static uint32_t s_accel;
static float s_speed_rpm = LIFT_STEPPER_DEFAULT_SPEED_RPM;

static uint32_t stepper_clamp_freq(uint32_t freq)
{
    if (freq < LIFT_STEPPER_MIN_FREQ_HZ)
    {
        return LIFT_STEPPER_MIN_FREQ_HZ;
    }
    if (freq > LIFT_STEPPER_MAX_FREQ_HZ)
    {
        return LIFT_STEPPER_MAX_FREQ_HZ;
    }
    return freq;
}

static uint32_t stepper_freq_to_arr(uint32_t freq)
{
    uint32_t arr;

    freq = stepper_clamp_freq(freq);
    arr = LIFT_STEPPER_TIMER_HZ / freq;
    if (arr == 0U)
    {
        arr = 1U;
    }
    return arr - 1U;
}

static void stepper_set_freq(uint32_t freq)
{
    uint32_t arr;

    freq = stepper_clamp_freq(freq);
    arr = stepper_freq_to_arr(freq);

    __HAL_TIM_SET_AUTORELOAD(&htim1, arr);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (arr + 1U) / 2U);
    s_freq = freq;
}

static void stepper_set_dir(LiftStepperDirection direction)
{
    GPIO_PinState level = (direction == LIFT_STEPPER_FORWARD) ?
                          LIFT_STEPPER_FORWARD_LEVEL :
                          (GPIO_PinState)!LIFT_STEPPER_FORWARD_LEVEL;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, level);
}

static void stepper_enable(uint8_t on)
{
    HAL_GPIO_WritePin(ENA_GPIO_Port, ENA_Pin,
                      (on != 0U) ? LIFT_STEPPER_ENABLE_LEVEL : LIFT_STEPPER_DISABLE_LEVEL);
}

static void stepper_stop_pulses(void)
{
    Tim1Mux_ReleaseStepper();
}

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

    stepper_stop_pulses();
    stepper_set_dir(LIFT_STEPPER_FORWARD);
    stepper_enable(0U);
    s_abort_requested = 0U;

    s_remaining = 0;
    s_freq = LIFT_STEPPER_MIN_FREQ_HZ;
    s_f_start = LIFT_STEPPER_START_FREQ_HZ;
    s_f_max = LIFT_STEPPER_START_FREQ_HZ;
    s_accel = LIFT_STEPPER_ACCEL_HZ_S;
    s_speed_rpm = LIFT_STEPPER_DEFAULT_SPEED_RPM;

    state = READY;
    return HAL_OK;
}

HAL_StatusTypeDef LiftStepper_SetSpeedRpm(float rpm)
{
    if (!isfinite(rpm) || (rpm < LIFT_STEPPER_MIN_SPEED_RPM) ||
        (rpm > LIFT_STEPPER_MAX_SPEED_RPM))
    {
        return HAL_ERROR;
    }

    s_speed_rpm = rpm;
    return HAL_OK;
}

float LiftStepper_GetSpeedRpm(void)
{
    return s_speed_rpm;
}

HAL_StatusTypeDef LiftStepper_Move(LiftStepperDirection direction, float angle_deg)
{
    double rounded_steps;
    uint32_t step_count;
    uint32_t f_max;
    uint32_t f_start;

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
    if (s_abort_requested != 0U)
    {
        return HAL_ERROR;
    }

    rounded_steps = (double)angle_deg * (double)LIFT_STEPPER_STEPS_PER_REV / 360.0 + 0.5;
    if (rounded_steps >= (double)INT32_MAX + 1.0)
    {
        return HAL_ERROR;
    }
    step_count = (uint32_t)rounded_steps;
    if (step_count == 0U)
    {
        return HAL_OK;
    }

    f_max = (uint32_t)(s_speed_rpm * (float)LIFT_STEPPER_STEPS_PER_REV / 60.0f + 0.5f);
    f_max = stepper_clamp_freq(f_max);
    f_start = (f_max < LIFT_STEPPER_START_FREQ_HZ) ? f_max : LIFT_STEPPER_START_FREQ_HZ;
    f_start = stepper_clamp_freq(f_start);

    stepper_set_dir(direction);
    stepper_enable(1U);
    HAL_Delay(LIFT_STEPPER_ENABLE_DELAY_MS);
    if (s_abort_requested != 0U)
    {
        return HAL_ERROR;
    }

    s_remaining = (int32_t)step_count;
    s_f_start = f_start;
    s_f_max = f_max;
    s_accel = LIFT_STEPPER_ACCEL_HZ_S;
    s_freq = f_start;

    Tim1Mux_TakeStepper(LIFT_STEPPER_TIMER_PSC, stepper_freq_to_arr(f_start));
    stepper_set_freq(f_start);

    state = MOVING;
    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3) != HAL_OK)
    {
        state = READY;
        stepper_stop_pulses();
        stepper_enable(0U);
        return HAL_ERROR;
    }
    if (HAL_TIM_Base_Start_IT(&htim1) != HAL_OK)
    {
        state = READY;
        stepper_stop_pulses();
        stepper_enable(0U);
        return HAL_ERROR;
    }

    while (state == MOVING)
    {
        if (s_abort_requested != 0U)
        {
            stepper_stop_pulses();
            state = READY;
            break;
        }
        HAL_Delay(1);
    }

    return (s_abort_requested != 0U) ? HAL_ERROR : HAL_OK;
}

void LiftStepper_Abort(void)
{
    s_abort_requested = 1U;
}

void LiftStepper_ClearAbort(void)
{
    s_abort_requested = 0U;
}

void LiftStepper_Stop(void)
{
    if (state == UNINITIALIZED)
    {
        return;
    }

    stepper_stop_pulses();
    state = READY;
}

void LiftStepper_Disable(void)
{
    LiftStepper_Stop();
    stepper_enable(0U);
}

uint8_t LiftStepper_IsBusy(void)
{
    return (state == MOVING) ? 1U : 0U;
}

void LiftStepper_OnPulse(void)
{
    uint32_t stop_dist;

    if (state != MOVING)
    {
        return;
    }
    if (s_abort_requested != 0U)
    {
        stepper_stop_pulses();
        state = READY;
        return;
    }

    s_remaining--;
    if (s_remaining <= 0)
    {
        stepper_stop_pulses();
        state = READY;
        return;
    }

    if (s_accel == 0U)
    {
        return;
    }

    /* 剩余步数不足以减速到起始频率时进入减速段。 */
    stop_dist = (uint32_t)(((uint64_t)s_freq * (uint64_t)s_freq) / (2ULL * s_accel));

    if (s_remaining <= (int32_t)stop_dist)
    {
        if (s_freq > s_f_start)
        {
            uint32_t dec = s_accel / s_freq;
            s_freq = (s_freq > (s_f_start + dec)) ? (s_freq - dec) : s_f_start;
        }
    }
    else if (s_freq < s_f_max)
    {
        uint32_t inc = s_accel / s_freq;
        s_freq = (s_freq + inc < s_f_max) ? (s_freq + inc) : s_f_max;
    }

    stepper_set_freq(s_freq);
}
