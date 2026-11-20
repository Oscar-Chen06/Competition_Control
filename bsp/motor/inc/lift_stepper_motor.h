#ifndef LIFT_STEPPER_MOTOR_H
#define LIFT_STEPPER_MOTOR_H

#include "stm32f1xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    LIFT_STEPPER_FORWARD = 0,
    LIFT_STEPPER_REVERSE = 1
} LiftStepperDirection;

/* ------------------------------ 机械 / 细分 ------------------------------- */
#define LIFT_STEPPER_MOTOR_STEP_ANGLE_DEG 1.8f
#define LIFT_STEPPER_MICROSTEP 4U /* TB6600 实际细分，需与拨码一致 */
#define LIFT_STEPPER_STEPS_PER_REV \
    ((uint32_t)(360.0f / LIFT_STEPPER_MOTOR_STEP_ANGLE_DEG * (float)LIFT_STEPPER_MICROSTEP))
#define LIFT_STEPPER_PULSES_PER_REV LIFT_STEPPER_STEPS_PER_REV /* 兼容旧名称 */

/* ------------------------------ 方向 / 使能 ------------------------------- */
#define LIFT_STEPPER_FORWARD_LEVEL GPIO_PIN_SET /* 正转方向电平，反向时改为 RESET */
#define LIFT_STEPPER_ENABLE_LEVEL GPIO_PIN_RESET  /* 低电平使能 */
#define LIFT_STEPPER_DISABLE_LEVEL GPIO_PIN_SET   /* 与使能电平相反 */
#define LIFT_STEPPER_ENABLE_DELAY_MS 10U          /* 使能后到第一个脉冲的等待时间 */

/* ------------------------------ 速度 / 加减速 ----------------------------- */
#define LIFT_STEPPER_DEFAULT_SPEED_RPM 90.0f
#define LIFT_STEPPER_MIN_SPEED_RPM 1.0f
#define LIFT_STEPPER_MAX_SPEED_RPM 120.0f

#define LIFT_STEPPER_TIMER_PSC 71U      /* 72 MHz/(71+1) = 1 MHz */
#define LIFT_STEPPER_TIMER_HZ 1000000UL /* TIM1 步进时基 */
#define LIFT_STEPPER_MIN_FREQ_HZ 20U
#define LIFT_STEPPER_MAX_FREQ_HZ 20000U
#define LIFT_STEPPER_START_FREQ_HZ 400U
#define LIFT_STEPPER_ACCEL_HZ_S 20000U

/* 在时钟和 GPIO 初始化后调用：PB12=DIR，PB15=ENA，PA10=TIM1_CH3 脉冲。
 * 初始化不发脉冲、不使能驱动器；TIM1 与 MG513 分时复用。
 */
HAL_StatusTypeDef LiftStepper_Init(void);

/* 相对转动：方向 + 非负角度，按微步数四舍五入，可超过一圈。
 * 阻塞到全部脉冲发完；结束后保持使能，直到调用 LiftStepper_Disable()。
 * HAL_OK 代表脉冲已发完，不代表已通过传感器确认位置。
 */
HAL_StatusTypeDef LiftStepper_Move(LiftStepperDirection direction, float angle_deg);

/* 设置默认转速，单位 rpm；立即返回。运动中调用只影响下一次动作。 */
HAL_StatusTypeDef LiftStepper_SetSpeedRpm(float rpm);

float LiftStepper_GetSpeedRpm(void);

/* 立即停止脉冲，保持当前使能状态；需要失能时调用 LiftStepper_Disable()。 */
void LiftStepper_Stop(void);

/* 立即停止并失能驱动器。MotorControl 只在全部流程 DONE 后调用。 */
void LiftStepper_Disable(void);

uint8_t LiftStepper_IsBusy(void);

/* TIM1 更新中断回调，由 HAL_TIM_PeriodElapsedCallback 分发。 */
void LiftStepper_OnPulse(void);

#ifdef __cplusplus
}
#endif

#endif /* LIFT_STEPPER_MOTOR_H */
