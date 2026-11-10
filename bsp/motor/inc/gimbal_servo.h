#ifndef GIMBAL_SERVO_H
#define GIMBAL_SERVO_H

#include "stm32f1xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* MG996R 的两端脉宽，实际 180° 行程需要上机校准。 */
#define GIMBAL_SERVO_MIN_PULSE_US 500U
#define GIMBAL_SERVO_MAX_PULSE_US 2500U

/* 正对输出轴观察顺时针；实际方向相反时将 1 改为 0。 */
#ifndef GIMBAL_SERVO_CW_DECREASES_PULSE
#define GIMBAL_SERVO_CW_DECREASES_PULSE 1U
#endif

#define GIMBAL_SERVO_HOME_SETTLE_MS 1000U /* 回起点后建议等待 1 秒 */

/* PA6 / TIM3_CH1：在 MX_TIM3_Init() 后调用，初始化不启动舵机。
 * TIM3 使用 72 MHz 时钟、PSC=71、ARR=19999，即 50 Hz、每计数 1 μs。
 * 两台舵机共用 TIM3，运行期间不要在外部停止或重置定时器。
 */
HAL_StatusTypeDef GimbalServo_Init(void);

/* 下发起点位置后立即返回；调用方需等待舵机转到起点。 */
HAL_StatusTypeDef GimbalServo_Home(void);

/* 在 Home() 并等待后调用，顺时针转到另一端；重复调用返回 HAL_ERROR。
 * 函数不等待动作结束，PWM 持续保持目标位置，无位置反馈。
 */
HAL_StatusTypeDef GimbalServo_RotateCW180(void);

#ifdef __cplusplus
}
#endif

#endif /* GIMBAL_SERVO_H */
