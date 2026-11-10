#ifndef GRIPPER_SERVO_H
#define GRIPPER_SERVO_H

#include "stm32f1xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 夹爪开合角度待定，先保留 0～180° 设置接口；脉宽端点需实机校准。 */
#define GRIPPER_SERVO_MIN_PULSE_US 500U
#define GRIPPER_SERVO_MAX_PULSE_US 2500U
#define GRIPPER_SERVO_MAX_ANGLE_DEG 180U

/* PA7 / TIM3_CH2：在 MX_TIM3_Init() 后调用，初始化不启动舵机。
 * TIM3 使用 72 MHz 时钟、PSC=71、ARR=19999，运行期间不要重置定时器。
 */
HAL_StatusTypeDef GripperServo_Init(void);

/* 设置绝对角度，范围 0～180°，越界或未初始化时返回 HAL_ERROR。
 * 下发后立即返回，PWM 持续保持位置；HAL_OK 不代表机械动作已完成。
 */
HAL_StatusTypeDef GripperServo_SetAngle(uint16_t angle);

#ifdef __cplusplus
}
#endif

#endif /* GRIPPER_SERVO_H */
