#include "pid.h"

#include <math.h>
#include <stddef.h>

/* 内外环共用同一套算法，只有字段选择不同 */
static float pid_compute(pid_t *p, uint8_t inner, float ref, float y, float dt)
{
    if (dt <= 0.0f)
    {
        return 0.0f;
    }

    float kp = (inner != 0U) ? p->kp_in : p->kp_out;
    float ki = (inner != 0U) ? p->ki_in : p->ki_out;
    float kd = (inner != 0U) ? p->kd_in : p->kd_out;
    float tau_d = (inner != 0U) ? p->tau_d_in : p->tau_d_out;
    float out_min = (inner != 0U) ? p->out_min_in : p->out_min_out;
    float out_max = (inner != 0U) ? p->out_max_in : p->out_max_out;
    float int_min = (inner != 0U) ? p->int_min_in : p->int_min_out;
    float int_max = (inner != 0U) ? p->int_max_in : p->int_max_out;
    float int_band = (inner != 0U) ? p->int_band_in : p->int_band_out;
    float jump_th = (inner != 0U) ? p->jump_th_in : p->jump_th_out;
    uint8_t reset_on_setpoint =
        (inner != 0U) ? p->reset_on_setpoint_in : p->reset_on_setpoint_out;

    uint8_t *sat = (inner != 0U) ? &p->sat_in : &p->sat_out;
    float *integ = (inner != 0U) ? &p->integ_in : &p->integ_out;
    float *d_state = (inner != 0U) ? &p->d_state_in : &p->d_state_out;
    float *y_prev = (inner != 0U) ? &p->y_prev_in : &p->y_prev_out;
    float *ref_prev = (inner != 0U) ? &p->ref_prev_in : &p->ref_prev_out;

    float e = ref - y;

    /* 给定突变：清积分，防阶跃爆冲 */
    if (reset_on_setpoint != 0U && fabsf(ref - *ref_prev) > jump_th)
    {
        *integ = 0.0f;
    }

    /* D 作用在测量值上并一阶滤波：Kd*s/(1+tau_d*s) * (-Y) */
    *d_state = (tau_d * (*d_state) - kd * (y - *y_prev)) / (tau_d + dt);
    float d_term = *d_state;

    /* 积分分离 + 上拍饱和时停止积分 */
    if (fabsf(e) <= int_band && *sat == 0U)
    {
        *integ += ki * e * dt;
    }
    if (*integ > int_max)
    {
        *integ = int_max;
    }
    else if (*integ < int_min)
    {
        *integ = int_min;
    }

    float u_raw = kp * e + *integ + d_term;
    float u = u_raw;
    if (u > out_max)
    {
        u = out_max;
    }
    else if (u < out_min)
    {
        u = out_min;
    }

    *sat = (u_raw > out_max || u_raw < out_min) ? 1U : 0U;
    *y_prev = y;
    *ref_prev = ref;

    return u;
}

void pid_init(pid_t *p)
{
    if (p == NULL)
    {
        return;
    }

    p->kp_out = PID_OUT_KP;
    p->ki_out = PID_OUT_KI;
    p->kd_out = PID_OUT_KD;
    p->tau_d_out = PID_OUT_TAU_D_S;
    p->out_min_out = PID_OUT_OUT_MIN;
    p->out_max_out = PID_OUT_OUT_MAX;
    p->int_min_out = PID_OUT_INT_MIN;
    p->int_max_out = PID_OUT_INT_MAX;
    p->int_band_out = PID_OUT_INT_BAND_DEG;
    p->jump_th_out = PID_OUT_JUMP_TH_DEG;
    p->reset_on_setpoint_out = (uint8_t)PID_OUT_RESET_ON_SETPOINT;

    p->kp_in = PID_IN_KP;
    p->ki_in = PID_IN_KI;
    p->kd_in = PID_IN_KD;
    p->tau_d_in = PID_IN_TAU_D_S;
    p->out_min_in = PID_IN_OUT_MIN;
    p->out_max_in = PID_IN_OUT_MAX;
    p->int_min_in = PID_IN_INT_MIN;
    p->int_max_in = PID_IN_INT_MAX;
    p->int_band_in = PID_IN_INT_BAND_DEG_S;
    p->jump_th_in = PID_IN_JUMP_TH_DEG_S;
    p->reset_on_setpoint_in = (uint8_t)PID_IN_RESET_ON_SETPOINT;

    pid_out_reset(p);
    pid_in_reset(p);
}

void pid_out_reset(pid_t *p)
{
    if (p == NULL)
    {
        return;
    }

    p->sat_out = 0U;
    p->integ_out = 0.0f;
    p->d_state_out = 0.0f;
    p->y_prev_out = 0.0f;
    p->ref_prev_out = 0.0f;
}

void pid_in_reset(pid_t *p)
{
    if (p == NULL)
    {
        return;
    }

    p->sat_in = 0U;
    p->integ_in = 0.0f;
    p->d_state_in = 0.0f;
    p->y_prev_in = 0.0f;
    p->ref_prev_in = 0.0f;
}

float pid_out_update(pid_t *p, float ref, float y, float dt)
{
    if (p == NULL)
    {
        return 0.0f;
    }
    return pid_compute(p, 0U, ref, y, dt);
}

float pid_in_update(pid_t *p, float ref, float y, float dt)
{
    if (p == NULL)
    {
        return 0.0f;
    }
    return pid_compute(p, 1U, ref, y, dt);
}

static float control_clampf(float v, float lo, float hi)
{
    if (v < lo)
    {
        return lo;
    }
    if (v > hi)
    {
        return hi;
    }
    return v;
}

static float control_ramp_toward(float current, float target, float step)
{
    if (step < 0.0f)
    {
        step = -step;
    }
    if (target > current + step)
    {
        return current + step;
    }
    if (target < current - step)
    {
        return current - step;
    }
    return target;
}

void control_init(control_t *c)
{
    if (c == NULL)
    {
        return;
    }

    c->mode = CTRL_MODE_POSITION;
    pid_init(&c->pid);

    c->angle_ref_deg = 0.0f;
    c->speed_cmd_deg_s = 0.0f;
    c->speed_ref_deg_s = 0.0f;
    c->speed_max_deg_s = PID_SPEED_MAX_DEG_S;
    c->ramp_step_deg_s = PID_SPEED_RAMP_DEG_S2 * PID_CTRL_DT_S;
    c->pwm_out = 0;

    c->pos_div = PID_POS_DIV;
    if (c->pos_div == 0U)
    {
        c->pos_div = 1U;
    }
    c->pos_cnt = 0U;
    c->sync_ref = 1U;
    c->angle_ref_valid = 0U;
}

void control_set_mode(control_t *c, ctrl_mode_t m)
{
    if (c == NULL || c->mode == m)
    {
        return;
    }

    c->mode = m;
    c->sync_ref = 1U;

    if (m == CTRL_MODE_VELOCITY)
    {
        /* 速度模式不保留位置目标，避免切回位置模式时突跳 */
        c->angle_ref_valid = 0U;
        pid_out_reset(&c->pid);
    }
    else
    {
        /* 位置模式：让外环在下一拍立即计算 */
        c->pos_cnt = 0U;
    }
}

void control_set_angle_deg(control_t *c, float angle_deg)
{
    if (c == NULL)
    {
        return;
    }

    c->angle_ref_deg = angle_deg;
    c->angle_ref_valid = 1U;
}

void control_set_angle_rounds(control_t *c, float rounds)
{
    control_set_angle_deg(c, rounds * MG513_ANGLE_DEG_PER_REV);
}

void control_set_speed_rpm(control_t *c, float rpm)
{
    if (c == NULL)
    {
        return;
    }

    c->speed_cmd_deg_s =
        control_clampf(rpm * PID_RPM_TO_DEG_S, -c->speed_max_deg_s, c->speed_max_deg_s);
}

int32_t control_update(control_t *c, float angle_deg, float speed_deg_s, float dt)
{
    if (c == NULL || dt <= 0.0f)
    {
        return 0;
    }

    /* 模式切换后的首个节拍：速度给定对齐实际，保证无扰 */
    if (c->sync_ref != 0U)
    {
        if (c->mode == CTRL_MODE_VELOCITY)
        {
            c->speed_ref_deg_s = speed_deg_s;
        }
        else
        {
            c->speed_ref_deg_s = 0.0f;
        }
        c->sync_ref = 0U;
    }

    if (c->mode == CTRL_MODE_VELOCITY)
    {
        /* 位置给定跟随实际角；外环不参与 */
        c->angle_ref_deg = angle_deg;
        c->speed_ref_deg_s =
            control_ramp_toward(c->speed_ref_deg_s, c->speed_cmd_deg_s,
                                c->ramp_step_deg_s * (dt / PID_CTRL_DT_S));
    }
    else
    {
        /* 没有显式目标时保持当前角度 */
        if (c->angle_ref_valid == 0U)
        {
            c->angle_ref_deg = angle_deg;
        }

        if (c->pos_cnt == 0U)
        {
            float u = pid_out_update(&c->pid, c->angle_ref_deg, angle_deg,
                                     dt * (float)c->pos_div);
            c->speed_ref_deg_s = control_clampf(u, -c->speed_max_deg_s, c->speed_max_deg_s);
        }

        c->pos_cnt++;
        if (c->pos_cnt >= c->pos_div)
        {
            c->pos_cnt = 0U;
        }
    }

    float pwm = pid_in_update(&c->pid, c->speed_ref_deg_s, speed_deg_s, dt);
    pwm = control_clampf(pwm, -((float)MG513_PWM_MAX), (float)MG513_PWM_MAX);
    c->pwm_out = (int32_t)lroundf(pwm);
    return c->pwm_out;
}
