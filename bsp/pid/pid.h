#ifndef PID_H
#define PID_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "mg513.h"

/* -------------------------------- 控制节拍 -------------------------------- */
#define PID_CTRL_DT_S 0.010f                               /* 内环采样周期 [s] = 10 ms */
#define PID_CTRL_FREQ_HZ 100U                              /* 内环频率 [Hz] */
#define PID_POS_DIV 5U                                     /* 外环分频：每 5 个内环周期跑一次外环 */
#define PID_POS_DT_S (PID_CTRL_DT_S * (float)PID_POS_DIV)  /* 外环周期 [s] = 50 ms */

/* ---------------------------------- 转速 ---------------------------------- */
#define PID_RPM_TO_DEG_S 6.0f         /* rpm * 6 = deg/s */
#define PID_DEG_S_TO_RPM (1.0f / 6.0f)
#define PID_SPEED_MAX_DEG_S 720.0f    /* 内环给定钳位 [deg/s] = 120 rpm */
#define PID_SPEED_RAMP_DEG_S2 1440.0f /* 速度模式给定斜坡加速度 [deg/s^2] */

/* --------------------------- 外环（位置环）PID ---------------------------- */
#define PID_OUT_KP 4.0f
#define PID_OUT_KI 1.8f
#define PID_OUT_KD 0.06f
#define PID_OUT_TAU_D_S 0.020f
#define PID_OUT_INT_BAND_DEG 50.0f
#define PID_OUT_JUMP_TH_DEG 5.0f
#define PID_OUT_RESET_ON_SETPOINT 1U
#define PID_OUT_INT_MIN (-PID_SPEED_MAX_DEG_S)
#define PID_OUT_INT_MAX (PID_SPEED_MAX_DEG_S)
#define PID_OUT_OUT_MIN (-PID_SPEED_MAX_DEG_S)
#define PID_OUT_OUT_MAX (PID_SPEED_MAX_DEG_S)

/* --------------------------- 内环（速度环）PID ---------------------------- */
#define PID_IN_KP 15.0f
#define PID_IN_KI 2.5f
#define PID_IN_KD 0.06f
#define PID_IN_TAU_D_S 0.03f
#define PID_IN_INT_BAND_DEG_S 180.0f
#define PID_IN_JUMP_TH_DEG_S 50.0f
#define PID_IN_RESET_ON_SETPOINT 1U
#define PID_IN_INT_MIN (-((float)MG513_PWM_MAX))
#define PID_IN_INT_MAX ((float)MG513_PWM_MAX)
#define PID_IN_OUT_MIN (-((float)MG513_PWM_MAX))
#define PID_IN_OUT_MAX ((float)MG513_PWM_MAX)

/* 一个结构体同时保存内外环：字段用 _out / _in 区分，两套参数与状态互不覆盖 */
typedef struct {
    /* ===== 外环（位置环）：输入 deg，输出 deg/s ===== */
    float kp_out, ki_out, kd_out;
    float tau_d_out;                /* D 滤波时间常数 [s] */
    float out_min_out, out_max_out; /* 输出钳位 [deg/s] */
    float int_min_out, int_max_out; /* 积分钳位 */
    float int_band_out;             /* 积分分离带 [deg] */
    float jump_th_out;              /* 给定突变阈值 [deg] */
    uint8_t reset_on_setpoint_out;  /* 1=给定突变清积分 */
    uint8_t sat_out;                /* 上拍输出是否饱和 */
    float integ_out;
    float d_state_out;
    float y_prev_out;
    float ref_prev_out;

    /* ===== 内环（速度环）：输入 deg/s，输出 pwm ===== */
    float kp_in, ki_in, kd_in;
    float tau_d_in;
    float out_min_in, out_max_in; /* 输出钳位 [pwm] */
    float int_min_in, int_max_in;
    float int_band_in; /* 积分分离带 [deg/s] */
    float jump_th_in;
    uint8_t reset_on_setpoint_in;
    uint8_t sat_in;
    float integ_in;
    float d_state_in;
    float y_prev_in;
    float ref_prev_in;
} pid_t;

/* 控制模式 */
typedef enum {
    CTRL_MODE_POSITION = 0, /* 位置模式：角度外环 + 速度内环 */
    CTRL_MODE_VELOCITY      /* 速度模式：旁路位置环，直接给速度内环 */
} ctrl_mode_t;

/* 串级控制器：两套 PID 参数/状态都放在 pid_t 里（_out / _in） */
typedef struct {
    ctrl_mode_t mode;

    pid_t pid;              /* 同一个结构体，内含外环与内环 */
    float angle_ref_deg;    /* 目标累计角度 [deg] */
    float speed_cmd_deg_s;  /* 速度模式目标 [deg/s]，由入口 rpm 换算 */
    float speed_ref_deg_s;  /* 当前内环给定 [deg/s] */
    float speed_max_deg_s;  /* 外环输出钳位 [deg/s] */
    float ramp_step_deg_s;  /* 速度模式每个 CTRL_DT 的斜坡步长 [deg/s] */
    int32_t pwm_out;        /* 最近一次输出的 pwm_cal（带符号） */

    uint32_t pos_div;        /* 外环分频 */
    uint32_t pos_cnt;        /* 外环分频计数 */
    uint8_t sync_ref;        /* 模式切换后首次更新：给定对齐实际值 */
    uint8_t angle_ref_valid; /* 用户是否显式设置过目标角度 */
} control_t;

/* 从 pid.h 装载内外环参数并清零状态 */
void pid_init(pid_t *p);

/* 只清对应环的运行时状态，保留参数 */
void pid_out_reset(pid_t *p);
void pid_in_reset(pid_t *p);

/* 计算：外环 deg -> deg/s；内环 deg/s -> pwm。返回值已按 out_min/out_max 钳位 */
float pid_out_update(pid_t *p, float ref, float y, float dt);
float pid_in_update(pid_t *p, float ref, float y, float dt);

/* 初始化：从 pid.h 装载参数，清零状态，默认进入 POSITION */
void control_init(control_t *c);

/* 切换模式。切入 VELOCITY 时旁路位置环；切入 POSITION 时保持当前角度 */
void control_set_mode(control_t *c, ctrl_mode_t m);

/* 设置目标角度：绝对累计角度或圈数，内部统一换算为 deg */
void control_set_angle_deg(control_t *c, float angle_deg);
void control_set_angle_rounds(control_t *c, float rounds);

/* 设置目标转速：外部单位 rpm，内部换算为 deg/s 并钳位 */
void control_set_speed_rpm(control_t *c, float rpm);

/* 控制节拍：按 PID_CTRL_DT_S 调用，返回四舍五入后的带符号 pwm */
int32_t control_update(control_t *c, float angle_deg, float speed_deg_s, float dt);

#ifdef __cplusplus
}
#endif

#endif /* PID_H */
