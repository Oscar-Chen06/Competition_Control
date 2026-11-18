#include "motor_control.h"
#include <math.h>
#include "gimbal_servo.h"
#include "gripper_servo.h"
#include "lift_stepper_motor.h"
#include "mg513.h"

static MotorControlState state = MOTOR_CONTROL_UNINITIALIZED;
static MotorControlError error = MOTOR_CONTROL_ERROR_NONE;
static float lift_depth_mm;

static HAL_StatusTypeDef fail(MotorControlError reason)
{
    MG513_Stop();
    error = reason;
    state = MOTOR_CONTROL_FAULT;
    return HAL_ERROR;
}

static uint8_t config_valid(void)
{
    const float depth = MOTOR_CONTROL_PICKUP_DEPTH_MM;
    const float travel = MOTOR_CONTROL_LIFT_TRAVEL_MM;
    const float carry = MOTOR_CONTROL_CARRY_LIFT_MM;
    const float wheel = MOTOR_CONTROL_WHEEL_DIAMETER_MM;
    const float distance = MOTOR_CONTROL_STATION_DISTANCE_MM;
    const float lead = MOTOR_CONTROL_SCREW_LEAD_MM;
    const float drop = MOTOR_CONTROL_PLATFORM_DROP_MM;

    return ((MOTOR_CONTROL_TRAVEL_SIGN == 1) || (MOTOR_CONTROL_TRAVEL_SIGN == -1)) &&
           ((MOTOR_CONTROL_LIFT_DOWN_FORWARD == 0) || (MOTOR_CONTROL_LIFT_DOWN_FORWARD == 1)) &&
           isfinite(depth) && isfinite(travel) && isfinite(carry) &&
           isfinite(wheel) && isfinite(distance) && isfinite(lead) && isfinite(drop) &&
           (wheel > 0.0f) && (distance > 0.0f) && (lead > 0.0f) && (drop >= 0.0f) &&
           (carry >= 5.0f) && (carry <= 10.0f) && (depth >= carry) &&
           (travel > 0.0f) && isfinite(depth + drop) && (depth + drop <= travel) &&
           (MOTOR_CONTROL_GRIP_OPEN_DEG >= 0) &&
           (MOTOR_CONTROL_GRIP_OPEN_DEG <= (int)GRIPPER_SERVO_MAX_ANGLE_DEG) &&
           (MOTOR_CONTROL_GRIP_CLOSED_DEG >= 0) &&
           (MOTOR_CONTROL_GRIP_CLOSED_DEG <= (int)GRIPPER_SERVO_MAX_ANGLE_DEG) &&
           (MOTOR_CONTROL_GRIP_OPEN_DEG != MOTOR_CONTROL_GRIP_CLOSED_DEG) &&
           (MOTOR_CONTROL_DRIVE_TIMEOUT_MS > 0U);
}

HAL_StatusTypeDef MotorControl_Init(void)
{
    if (state != MOTOR_CONTROL_UNINITIALIZED)
        return state == MOTOR_CONTROL_READY ? HAL_OK : HAL_ERROR;
    if (__get_IPSR() != 0U || __get_PRIMASK() != 0U)
        return fail(MOTOR_CONTROL_ERROR_DRIVER);
    if (LiftStepper_Init() != HAL_OK)
        return fail(MOTOR_CONTROL_ERROR_DRIVER);
    if (MOTOR_CONTROL_CALIBRATED != 1U)
        return fail(MOTOR_CONTROL_ERROR_NOT_CALIBRATED);
    if (!config_valid())
        return fail(MOTOR_CONTROL_ERROR_CONFIG);
    if (GripperServo_Init() != HAL_OK || GimbalServo_Init() != HAL_OK ||
        GimbalServo_ConfirmHome() != HAL_OK || MG513_Init() != HAL_OK)
        return fail(MOTOR_CONTROL_ERROR_DRIVER);

    lift_depth_mm = 0.0f;
    state = MOTOR_CONTROL_READY;
    return HAL_OK;
}

static HAL_StatusTypeDef grip(uint16_t angle)
{
    if (GripperServo_SetAngle(angle) != HAL_OK)
        return fail(MOTOR_CONTROL_ERROR_DRIVER);
    HAL_Delay(MOTOR_CONTROL_GRIP_SETTLE_MS);
    return HAL_OK;
}

static HAL_StatusTypeDef lift_to(float target_mm)
{
    if (!isfinite(target_mm) || target_mm < 0.0f || target_mm > MOTOR_CONTROL_LIFT_TRAVEL_MM)
        return fail(MOTOR_CONTROL_ERROR_CONFIG);
    const float delta = target_mm - lift_depth_mm;
    const uint8_t forward = delta >= 0.0f ? MOTOR_CONTROL_LIFT_DOWN_FORWARD : !MOTOR_CONTROL_LIFT_DOWN_FORWARD;
    if (LiftStepper_Move(forward ? LIFT_STEPPER_FORWARD : LIFT_STEPPER_REVERSE,
                         fabsf(delta) / MOTOR_CONTROL_SCREW_LEAD_MM * 360.0f) != HAL_OK)
        return fail(MOTOR_CONTROL_ERROR_DRIVER);
    lift_depth_mm = target_mm;
    return HAL_OK;
}

static HAL_StatusTypeDef drive_to(float distance_mm)
{
    const float angle = MOTOR_CONTROL_TRAVEL_SIGN * distance_mm /
                        (3.14159265358979323846f * MOTOR_CONTROL_WHEEL_DIAMETER_MM) * 360.0f;
    if (MG513_MoveTo(0, angle) != HAL_OK)
        return fail(MOTOR_CONTROL_ERROR_DRIVER);
    const uint32_t started_ms = HAL_GetTick();
    for (;;)
    {
        if ((uint32_t)(HAL_GetTick() - started_ms) >= MOTOR_CONTROL_DRIVE_TIMEOUT_MS)
            return fail(MOTOR_CONTROL_ERROR_TIMEOUT);
        if (MG513_MoveDone()) break;
        HAL_Delay(1); /* 保持中断运行，TIM2 继续执行底盘 PID。 */
    }
    MG513_Stop();
    HAL_Delay(MOTOR_CONTROL_DRIVE_SETTLE_MS);
    return HAL_OK;
}

HAL_StatusTypeDef MotorControl_RunOnce(void)
{
    if (state != MOTOR_CONTROL_READY) return HAL_ERROR;
    if (__get_IPSR() != 0U || __get_PRIMASK() != 0U)
        return fail(MOTOR_CONTROL_ERROR_DRIVER);
    state = MOTOR_CONTROL_RUNNING;
    HAL_Delay(MOTOR_CONTROL_START_DELAY_MS);

    /* 取货：每个函数等待完成才返回；任何失败立即结束，不继续下一动作。 */
    if (grip(MOTOR_CONTROL_GRIP_OPEN_DEG) != HAL_OK) return HAL_ERROR;
    if (lift_to(MOTOR_CONTROL_PICKUP_DEPTH_MM) != HAL_OK) return HAL_ERROR;
    if (grip(MOTOR_CONTROL_GRIP_CLOSED_DEG) != HAL_OK) return HAL_ERROR;
    if (lift_to(MOTOR_CONTROL_PICKUP_DEPTH_MM - MOTOR_CONTROL_CARRY_LIFT_MM) != HAL_OK) return HAL_ERROR;

    /* 运送到台2，停车后才旋转。初始化时不转动云台。 */
    if (drive_to(MOTOR_CONTROL_STATION_DISTANCE_MM) != HAL_OK) return HAL_ERROR;
    if (GimbalServo_RotateCW180() != HAL_OK) return fail(MOTOR_CONTROL_ERROR_DRIVER);
    HAL_Delay(MOTOR_CONTROL_GIMBAL_SETTLE_MS);

    /* 放货、空爪升到最高处、返回起点。 */
    if (lift_to(MOTOR_CONTROL_PICKUP_DEPTH_MM + MOTOR_CONTROL_PLATFORM_DROP_MM) != HAL_OK) return HAL_ERROR;
    if (grip(MOTOR_CONTROL_GRIP_OPEN_DEG) != HAL_OK) return HAL_ERROR;
    if (lift_to(0.0f) != HAL_OK) return HAL_ERROR;
    if (drive_to(0.0f) != HAL_OK) return HAL_ERROR;
    state = MOTOR_CONTROL_DONE;
    return HAL_OK;
}

void MotorControl_Stop(void)
{
    MG513_Stop();
    if (state != MOTOR_CONTROL_FAULT && state != MOTOR_CONTROL_DONE)
        state = MOTOR_CONTROL_STOPPED;
}

MotorControlState MotorControl_GetState(void) { return state; }
MotorControlError MotorControl_GetError(void) { return error; }
