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

/* 原步距 1.8°，驱动器设为 4 细分：每圈 800 脉冲，每脉冲 0.45°。 */
#define LIFT_STEPPER_PULSES_PER_REV 800U
#define LIFT_STEPPER_FORWARD_LEVEL GPIO_PIN_SET  /* 实际方向相反时改为 RESET */
#define LIFT_STEPPER_ENABLE_LEVEL GPIO_PIN_RESET /* TB6600：低电平使能 */
#define LIFT_STEPPER_PULSE_DELAY_MS 1U          /* 高、低电平各延时 1 ms */
#define LIFT_STEPPER_ENABLE_DELAY_MS 10U        /* 方向、使能设置后等待 10 ms */

/* 在时钟和 GPIO 初始化后调用：PA10=PUL，PB12=DIR，PB15=ENA。
 * 初始化不发脉冲；使用 HAL_Delay() 延时，不占用舵机的 TIM3。
 */
HAL_StatusTypeDef LiftStepper_Init(void);

/* 相对转动：方向 + 非负角度，角度按 0.45° 四舍五入，可超过一圈。
 * 例：LiftStepper_Move(LIFT_STEPPER_FORWARD, 180.0f); // 正转 400 脉冲
 * 阻塞到脉冲发完，只在主循环调用；结束后保持使能，维持升降位置。
 * 固定延时发脉冲，不加减速；增大 PULSE_DELAY_MS 可降低速度。
 * HAL_Delay() 依赖系统节拍，运行期间需保持节拍中断工作。
 * 延时参数为毫秒，实际速度受节拍取整和中断影响，会比理论值慢。
 * HAL_OK 代表脉冲已发完，不代表已通过传感器确认位置。
 */
HAL_StatusTypeDef LiftStepper_Move(LiftStepperDirection direction, float angle_deg);

#ifdef __cplusplus
}
#endif

#endif /* LIFT_STEPPER_MOTOR_H */
