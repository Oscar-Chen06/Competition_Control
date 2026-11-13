#ifndef MG513_H
#define MG513_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ------------------------------- 硬件定时器 ------------------------------- */
#define MG513_PWM_TIM_PSC 0U       /* TIM4 时钟 72 MHz */
#define MG513_PWM_ARR 7199U        /* 72 MHz/(7199+1) = 10 kHz */
#define MG513_PWM_FREQ_HZ 10000U
#define MG513_CTRL_TIM_PSC 71U     /* TIM2：72 MHz/(71+1) = 1 MHz */
#define MG513_CTRL_TIM_ARR 9999U   /* 1 MHz/10000 = 100 Hz = 10 ms */

/* ---------------------------------- PWM ---------------------------------- */
#define MG513_PWM_MAX 7000         /* pwm 绝对上限，需 < MG513_PWM_ARR */
#define MG513_PWM_DEADBAND 50      /* 小于此值直接当 0 */

/* ------------------------------ 编码器 / 机械 ----------------------------- */
/* ENC_PPR 为电机轴一圈计数（已含 TIM1 四倍频）；CNT_PER_OUTPUT_REV 必须实测标定。 */
#define MG513_ENC_PPR 52.0f
#define MG513_GEAR_RATIO 30.0f
#define MG513_CNT_PER_OUTPUT_REV (MG513_ENC_PPR * MG513_GEAR_RATIO)
#define MG513_ANGLE_DEG_PER_REV 360.0f

/* ------------------------------ 速度 / 滤波 ------------------------------- */
#define MG513_SPEED_TAU_S 0.020f /* 速度反馈一阶低通时间常数 [s] */

/* ------------------------------ 极性 / 方向 ------------------------------- */
#define MG513_ENCODER_INVERT 1 /* 编码器计数方向与电机正转相反时置 1 */
#define MG513_DIR_INVERT 0     /* 方向接反时置 1 */
#define MG513_EN_ACTIVE_LOW 0  /* 0=高电平使能（PB5）；低电平使能时改为 1 */

/* ------------------------------ 动作完成判定 ------------------------------ */
#define MG513_DONE_BAND_DEG 2.0f
#define MG513_DONE_SPEED_DEG_S 10.0f

/* 累计位置：总角度 = rounds×360° + deg，deg 可正负 */
typedef struct {
    int32_t rounds;
    float deg;
} mg513_position_t;

/* 电机 + 编码器：硬件映射、状态估计、PWM/方向/使能控制 */
typedef struct {
    GPIO_TypeDef *dir1_port;
    uint16_t dir1_pin;
    GPIO_TypeDef *dir2_port;
    uint16_t dir2_pin;
    GPIO_TypeDef *en_port;
    uint16_t en_pin;

    TIM_HandleTypeDef *htim_pwm;
    uint32_t pwm_channel;
    TIM_HandleTypeDef *htim_enc;

    float cnt_per_output_rev; /* 输出轴一圈的编码器计数 */
    float speed_tau;          /* 速度反馈一阶低通时间常数 [s] */

    int32_t enc_ext;  /* 32 位累计计数 */
    int16_t enc_last; /* 上拍 16 位计数 */
    float angle_deg;  /* 累计角度 [deg] */
    float speed_deg_s; /* 输出轴角速度 [deg/s] */
    int8_t last_dir;   /* 上次方向：-1 未知，0 反转，1 正转 */
} mg513_t;

/* 初始化并启动编码器、PWM、TIM2 10 ms 节拍；驱动保持失能，首次位置指令时才使能。
 * 硬件：TIM4_CH1/PB6 PWM，PB5 高电平使能，PB0/PB1 方向，TIM1 PA8/PA9 编码器。
 */
HAL_StatusTypeDef MG513_Init(void);

/* 位置模式：转到绝对目标角度，rounds 圈 + deg 度（累计角度） */
HAL_StatusTypeDef MG513_MoveTo(int32_t rounds, float deg);

/* 位置模式：相对当前位置转动 rounds 圈 + deg 度（可正负） */
HAL_StatusTypeDef MG513_MoveRelative(int32_t rounds, float deg);

/* 立即停止：目标设为当前位置，清 PWM 并失能驱动 */
void MG513_Stop(void);

/* 当前位置动作是否到位（角度误差和速度都在阈值内） */
uint8_t MG513_MoveDone(void);

/* 返回累计位置：总角度 = rounds×360° + deg */
mg513_position_t MG513_GetPosition(void);

#ifdef __cplusplus
}
#endif

#endif /* MG513_H */
