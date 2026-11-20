#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "stm32f1xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MOTOR_CONTROL_CALIBRATED
#define MOTOR_CONTROL_CALIBRATED 1U
#endif
#ifndef MOTOR_CONTROL_TRAVEL_SIGN
#define MOTOR_CONTROL_TRAVEL_SIGN 1
#endif
#ifndef MOTOR_CONTROL_LIFT_DOWN_FORWARD
#define MOTOR_CONTROL_LIFT_DOWN_FORWARD (0)
#endif
#ifndef MOTOR_CONTROL_PICKUP_DEPTH_MM
#define MOTOR_CONTROL_PICKUP_DEPTH_MM (50.0f)
#endif
#ifndef MOTOR_CONTROL_LIFT_TRAVEL_MM
#define MOTOR_CONTROL_LIFT_TRAVEL_MM (100.0f)
#endif
#ifndef MOTOR_CONTROL_GRIP_OPEN_DEG
#define MOTOR_CONTROL_GRIP_OPEN_DEG (100.0f)
#endif
#ifndef MOTOR_CONTROL_GRIP_CLOSED_DEG
#define MOTOR_CONTROL_GRIP_CLOSED_DEG (0.0f)
#endif
#ifndef MOTOR_CONTROL_CARRY_LIFT_MM
#define MOTOR_CONTROL_CARRY_LIFT_MM 10.0f
#endif

#define MOTOR_CONTROL_WHEEL_DIAMETER_MM 48.0f
#define MOTOR_CONTROL_STATION_DISTANCE_MM 270.0f
#define MOTOR_CONTROL_SCREW_LEAD_MM 8.0f
#define MOTOR_CONTROL_PLATFORM_DROP_MM 40.0f
#define MOTOR_CONTROL_START_DELAY_MS 3000U
#define MOTOR_CONTROL_GRIP_SETTLE_MS 1000U
#define MOTOR_CONTROL_GIMBAL_SETTLE_MS 1500U
#define MOTOR_CONTROL_DRIVE_SETTLE_MS 200U
#define MOTOR_CONTROL_DRIVE_TIMEOUT_MS 15000U

typedef enum {
    MOTOR_CONTROL_UNINITIALIZED = 0,
    MOTOR_CONTROL_READY,
    MOTOR_CONTROL_RUNNING,
    MOTOR_CONTROL_DONE,
    MOTOR_CONTROL_STOPPED,
    MOTOR_CONTROL_FAULT
} MotorControlState;

typedef enum {
    MOTOR_CONTROL_ERROR_NONE = 0,
    MOTOR_CONTROL_ERROR_NOT_CALIBRATED,
    MOTOR_CONTROL_ERROR_CONFIG,
    MOTOR_CONTROL_ERROR_DRIVER,
    MOTOR_CONTROL_ERROR_TIMEOUT
} MotorControlError;

HAL_StatusTypeDef MotorControl_Init(void);
/* 阻塞执行一次，完成或失败才返回。只允许主程序调用且保持中断开启。
 * 成功后状态为 DONE；再次执行前需调用 MotorControl_Rearm()。
 * 停止或故障后不能自动重新运行，需复位并人工恢复初始位置。
 */
HAL_StatusTypeDef MotorControl_RunOnce(void);
/* 仅在 DONE 后调用：确认无动作、清停止标志并回到 READY，供下一次 RunOnce。 */
HAL_StatusTypeDef MotorControl_Rearm(void);
/* 请求中止当前运行流程；只置位，可在 EXTI 中断中调用。 */
void MotorControl_RequestStop(void);
/* 主程序停机入口，不是异步急停：阻塞运行期间主循环不能调用本函数。
 * 不松爪、不关闭舵机 PWM；运行中保持升降力矩，全部 DONE 后才失能；不得在中断中调用。
 */
void MotorControl_Stop(void);
MotorControlState MotorControl_GetState(void);
MotorControlError MotorControl_GetError(void);

#ifdef __cplusplus
}
#endif
#endif
