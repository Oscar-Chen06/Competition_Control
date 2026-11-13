#include "mg513.h"

#include <math.h>
#include <stddef.h>

#include "pid.h"
#include "tim.h"

static mg513_t s_motor;
static control_t s_ctrl;
/* 三种状态：未初始化、已初始化待命、控制运行中。 */
static volatile enum { UNINITIALIZED, READY, RUNNING } state;

static uint32_t mg513_enter_critical(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void mg513_exit_critical(uint32_t primask)
{
    __set_PRIMASK(primask);
}

static void motor_set_dir(mg513_t *m, uint8_t forward)
{
#if MG513_DIR_INVERT
    forward = (uint8_t)!forward;
#endif

    if (forward != 0U)
    {
        HAL_GPIO_WritePin(m->dir1_port, m->dir1_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(m->dir2_port, m->dir2_pin, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(m->dir1_port, m->dir1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(m->dir2_port, m->dir2_pin, GPIO_PIN_SET);
    }
}

static void motor_init(mg513_t *m)
{
    if (m == NULL)
    {
        return;
    }

    m->dir1_port = motor_dir1_GPIO_Port;
    m->dir1_pin = motor_dir1_Pin;
    m->dir2_port = motor_dir2_GPIO_Port;
    m->dir2_pin = motor_dir2_Pin;
    m->en_port = motor_enable_GPIO_Port;
    m->en_pin = motor_enable_Pin;

    m->htim_pwm = &htim4;
    m->pwm_channel = TIM_CHANNEL_1;
    m->htim_enc = &htim1;

    m->cnt_per_output_rev = MG513_CNT_PER_OUTPUT_REV;
    m->speed_tau = MG513_SPEED_TAU_S;

    m->enc_last = (int16_t)__HAL_TIM_GET_COUNTER(m->htim_enc);
    m->enc_ext = 0;
    m->angle_deg = 0.0f;
    m->speed_deg_s = 0.0f;

    m->last_dir = -1;
    HAL_GPIO_WritePin(m->dir1_port, m->dir1_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(m->dir2_port, m->dir2_pin, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(m->htim_pwm, m->pwm_channel, 0U);
}

static void motor_update(mg513_t *m, float dt)
{
    if (m == NULL || dt <= 0.0f)
    {
        return;
    }

    /* 16 位计数器有符号差，自动处理 65535 -> 0 回绕 */
    int16_t now = (int16_t)__HAL_TIM_GET_COUNTER(m->htim_enc);
    int16_t delta = (int16_t)(now - m->enc_last);
    m->enc_last = now;

#if MG513_ENCODER_INVERT
    delta = (int16_t)-delta;
#endif

    m->enc_ext += (int32_t)delta;

    if (m->cnt_per_output_rev <= 0.0f)
    {
        m->angle_deg = 0.0f;
        m->speed_deg_s = 0.0f;
        return;
    }

    float deg_per_cnt = MG513_ANGLE_DEG_PER_REV / m->cnt_per_output_rev;
    float speed_raw = (float)delta * deg_per_cnt / dt;
    float alpha;

    m->angle_deg = (float)m->enc_ext * deg_per_cnt;
    alpha = (m->speed_tau <= 0.0f) ? 1.0f : dt / (m->speed_tau + dt);
    m->speed_deg_s += alpha * (speed_raw - m->speed_deg_s);
}

static void motor_set_pwm(mg513_t *m, int32_t pwm_cal)
{
    if (m == NULL)
    {
        return;
    }

    /* 运行期再钳一次：即使 ARR 被改错，也绝不让 CCR > ARR 导致常高 */
    uint32_t arr = m->htim_pwm->Instance->ARR;
    int32_t max_duty = (arr < (uint32_t)MG513_PWM_MAX) ? (int32_t)arr : (int32_t)MG513_PWM_MAX;

    if (pwm_cal > max_duty)
    {
        pwm_cal = max_duty;
    }
    else if (pwm_cal < -max_duty)
    {
        pwm_cal = -max_duty;
    }

    if ((pwm_cal > -MG513_PWM_DEADBAND) && (pwm_cal < MG513_PWM_DEADBAND))
    {
        pwm_cal = 0;
    }

    if (pwm_cal == 0)
    {
        __HAL_TIM_SET_COMPARE(m->htim_pwm, m->pwm_channel, 0U);
        return;
    }

    uint8_t forward = (pwm_cal > 0) ? 1U : 0U;
    if (pwm_cal < 0)
    {
        pwm_cal = -pwm_cal;
    }

    /* 换向：先关输出一拍再切方向，下一拍才给占空比，避免带载直接反向 */
    if (m->last_dir != (int8_t)forward)
    {
        __HAL_TIM_SET_COMPARE(m->htim_pwm, m->pwm_channel, 0U);
        motor_set_dir(m, forward);
        m->last_dir = (int8_t)forward;
        return;
    }

    __HAL_TIM_SET_COMPARE(m->htim_pwm, m->pwm_channel, (uint32_t)pwm_cal);
}

static void motor_enable(mg513_t *m, uint8_t on)
{
    if (m == NULL)
    {
        return;
    }

#if MG513_EN_ACTIVE_LOW
    GPIO_PinState level = (on != 0U) ? GPIO_PIN_RESET : GPIO_PIN_SET;
#else
    GPIO_PinState level = (on != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET;
#endif

    HAL_GPIO_WritePin(m->en_port, m->en_pin, level);
}

static uint8_t mg513_position_done(void)
{
    if (s_ctrl.angle_ref_valid == 0U)
    {
        return 0U;
    }

    return (fabsf(s_ctrl.angle_ref_deg - s_motor.angle_deg) <= MG513_DONE_BAND_DEG &&
            fabsf(s_motor.speed_deg_s) <= MG513_DONE_SPEED_DEG_S)
               ? 1U
               : 0U;
}

static void mg513_tick(void)
{
    if (state == UNINITIALIZED)
    {
        return;
    }

    motor_update(&s_motor, PID_CTRL_DT_S);

    if (state != RUNNING)
    {
        motor_set_pwm(&s_motor, 0);
        return;
    }

    if (mg513_position_done() != 0U)
    {
        motor_set_pwm(&s_motor, 0);
        motor_enable(&s_motor, 0U);
        state = READY;
        return;
    }

    int32_t pwm = control_update(&s_ctrl, s_motor.angle_deg, s_motor.speed_deg_s, PID_CTRL_DT_S);
    motor_set_pwm(&s_motor, pwm);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        mg513_tick();
    }
}

HAL_StatusTypeDef MG513_Init(void)
{
    if (state != UNINITIALIZED)
    {
        return HAL_OK;
    }

    if ((htim1.Instance != TIM1) || (htim2.Instance != TIM2) || (htim4.Instance != TIM4))
    {
        return HAL_ERROR;
    }
    if ((htim2.Init.Prescaler != MG513_CTRL_TIM_PSC) ||
        (htim4.Init.Prescaler != MG513_PWM_TIM_PSC))
    {
        return HAL_ERROR;
    }

    motor_init(&s_motor);
    control_init(&s_ctrl);
    motor_enable(&s_motor, 0U);
    __HAL_TIM_SET_AUTORELOAD(&htim2, MG513_CTRL_TIM_ARR);
    __HAL_TIM_SET_AUTORELOAD(&htim4, MG513_PWM_ARR);

    if (HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1) != HAL_OK)
    {
        (void)HAL_TIM_Encoder_Stop(&htim1, TIM_CHANNEL_ALL);
        return HAL_ERROR;
    }

    state = READY;

    if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK)
    {
        state = UNINITIALIZED;
        motor_enable(&s_motor, 0U);
        (void)HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
        (void)HAL_TIM_Encoder_Stop(&htim1, TIM_CHANNEL_ALL);
        return HAL_ERROR;
    }

    return HAL_OK;
}

static HAL_StatusTypeDef mg513_start_position(float target_deg)
{
    if ((state == UNINITIALIZED) || !isfinite(target_deg))
    {
        return HAL_ERROR;
    }

    uint32_t primask = mg513_enter_critical();
    if (state == READY)
    {
        control_init(&s_ctrl);
    }
    control_set_mode(&s_ctrl, CTRL_MODE_POSITION);
    control_set_angle_deg(&s_ctrl, target_deg);
    motor_enable(&s_motor, 1U);
    state = RUNNING;
    mg513_exit_critical(primask);
    return HAL_OK;
}

HAL_StatusTypeDef MG513_MoveTo(int32_t rounds, float deg)
{
    return mg513_start_position((float)rounds * MG513_ANGLE_DEG_PER_REV + deg);
}

HAL_StatusTypeDef MG513_MoveRelative(int32_t rounds, float deg)
{
    if ((state == UNINITIALIZED) || !isfinite(deg))
    {
        return HAL_ERROR;
    }

    uint32_t primask = mg513_enter_critical();
    float target = s_motor.angle_deg + (float)rounds * MG513_ANGLE_DEG_PER_REV + deg;
    mg513_exit_critical(primask);
    return mg513_start_position(target);
}

void MG513_Stop(void)
{
    if (state == UNINITIALIZED)
    {
        return;
    }

    uint32_t primask = mg513_enter_critical();
    control_set_mode(&s_ctrl, CTRL_MODE_POSITION);
    control_set_angle_deg(&s_ctrl, s_motor.angle_deg);
    pid_out_reset(&s_ctrl.pid);
    pid_in_reset(&s_ctrl.pid);
    motor_set_pwm(&s_motor, 0);
    motor_enable(&s_motor, 0U);
    state = READY;
    mg513_exit_critical(primask);
}

uint8_t MG513_MoveDone(void)
{
    if (state == UNINITIALIZED)
    {
        return 0U;
    }

    uint32_t primask = mg513_enter_critical();
    uint8_t done = mg513_position_done();
    mg513_exit_critical(primask);
    return done;
}

mg513_position_t MG513_GetPosition(void)
{
    mg513_position_t pos = {0, 0.0f};

    if (state == UNINITIALIZED)
    {
        return pos;
    }

    uint32_t primask = mg513_enter_critical();
    float angle = s_motor.angle_deg;
    mg513_exit_critical(primask);

    pos.rounds = (int32_t)(angle / MG513_ANGLE_DEG_PER_REV);
    pos.deg = angle - (float)pos.rounds * MG513_ANGLE_DEG_PER_REV;
    return pos;
}
