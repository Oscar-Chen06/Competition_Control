#include "motor_control.h"
#include <math.h>
#include "gimbal_servo.h"
#include "gripper_servo.h"
#include "lift_stepper_motor.h"
#include "mg513.h"

static volatile MotorControlState state = MOTOR_CONTROL_UNINITIALIZED;
static MotorControlError error = MOTOR_CONTROL_ERROR_NONE;
static volatile uint8_t stop_requested;
static float lift_depth_mm;

static HAL_StatusTypeDef fail(MotorControlError reason)
{
    MG513_Stop();
    error = reason;
    state = MOTOR_CONTROL_FAULT;
    return HAL_ERROR;
}

void MotorControl_RequestStop(void)
{
    if (state == MOTOR_CONTROL_RUNNING)
    {
        stop_requested = 1U;
        LiftStepper_Abort();
    }
}

static HAL_StatusTypeDef abort_if_requested(void)
{
    if (stop_requested == 0U)
    {
        return HAL_OK;
    }

    MG513_Stop();
    LiftStepper_Stop();
    state = MOTOR_CONTROL_STOPPED;
    return HAL_ERROR;
}

static HAL_StatusTypeDef delay_with_stop(uint32_t delay_ms)
{
    const uint32_t started_ms = HAL_GetTick();

    while ((uint32_t)(HAL_GetTick() - started_ms) < delay_ms)
    {
        if (stop_requested != 0U)
        {
            return HAL_ERROR;
        }
        HAL_Delay(1);
    }
    return HAL_OK;
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
    if (delay_with_stop(MOTOR_CONTROL_GRIP_SETTLE_MS) != HAL_OK)
        return abort_if_requested();
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
    {
        if (stop_requested != 0U)
            return abort_if_requested();
        return fail(MOTOR_CONTROL_ERROR_DRIVER);
    }
    if (abort_if_requested() != HAL_OK)
        return HAL_ERROR;
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
        if (abort_if_requested() != HAL_OK)
            return HAL_ERROR;
        if ((uint32_t)(HAL_GetTick() - started_ms) >= MOTOR_CONTROL_DRIVE_TIMEOUT_MS)
            return fail(MOTOR_CONTROL_ERROR_TIMEOUT);
        if (MG513_MoveDone()) break;
        HAL_Delay(1); /* 保持中断运行，TIM4 继续执行底盘 PID。 */
    }
    MG513_Stop();
    if (delay_with_stop(MOTOR_CONTROL_DRIVE_SETTLE_MS) != HAL_OK)
        return abort_if_requested();
    return HAL_OK;
}

HAL_StatusTypeDef MotorControl_RunOnce(void)
{
    if (state != MOTOR_CONTROL_READY) return HAL_ERROR;
    if (__get_IPSR() != 0U || __get_PRIMASK() != 0U)
        return fail(MOTOR_CONTROL_ERROR_DRIVER);

    stop_requested = 0U;
    LiftStepper_ClearAbort();
    state = MOTOR_CONTROL_RUNNING;
    if (delay_with_stop(MOTOR_CONTROL_START_DELAY_MS) != HAL_OK)
        return abort_if_requested();

    /* 初始状态：先确认夹爪张开，再把云台回到 0° 起点。 */
    if (grip(MOTOR_CONTROL_GRIP_OPEN_DEG) != HAL_OK) return HAL_ERROR;
    if (abort_if_requested() != HAL_OK) return HAL_ERROR;
    if (GimbalServo_Home() != HAL_OK) return fail(MOTOR_CONTROL_ERROR_DRIVER);
    if (abort_if_requested() != HAL_OK) return HAL_ERROR;

    /* 取货：每个函数等待完成才返回；任何失败立即结束，不继续下一动作。 */
    if (lift_to(MOTOR_CONTROL_PICKUP_DEPTH_MM) != HAL_OK) return HAL_ERROR;
    if (grip(MOTOR_CONTROL_GRIP_CLOSED_DEG) != HAL_OK) return HAL_ERROR;
    if (lift_to(MOTOR_CONTROL_PICKUP_DEPTH_MM - MOTOR_CONTROL_CARRY_LIFT_MM) != HAL_OK) return HAL_ERROR;

    /* 运送到台2，停车后才旋转；启动时云台已确认在 0°。 */
    if (drive_to(MOTOR_CONTROL_STATION_DISTANCE_MM) != HAL_OK) return HAL_ERROR;
    if (GimbalServo_RotateCW180() != HAL_OK) return fail(MOTOR_CONTROL_ERROR_DRIVER);
    if (delay_with_stop(MOTOR_CONTROL_GIMBAL_SETTLE_MS) != HAL_OK)
        return abort_if_requested();

    /* 放货、空爪升到最高处、返回起点。 */
    if (lift_to(MOTOR_CONTROL_PICKUP_DEPTH_MM + MOTOR_CONTROL_PLATFORM_DROP_MM) != HAL_OK) return HAL_ERROR;
    if (grip(MOTOR_CONTROL_GRIP_OPEN_DEG) != HAL_OK) return HAL_ERROR;
    if (lift_to(0.0f) != HAL_OK) return HAL_ERROR;
    if (drive_to(0.0f) != HAL_OK) return HAL_ERROR;
    if (GimbalServo_Home() != HAL_OK) return fail(MOTOR_CONTROL_ERROR_DRIVER);
    if (abort_if_requested() != HAL_OK) return HAL_ERROR;

    /* 全部动作完成后才解除升降步进使能；运行中保持力矩。 */
    LiftStepper_Disable();
    state = MOTOR_CONTROL_DONE;
    return HAL_OK;
}

HAL_StatusTypeDef MotorControl_Rearm(void)
{
    if (state != MOTOR_CONTROL_DONE || LiftStepper_IsBusy() != 0U)
        return HAL_ERROR;

    MG513_Stop();
    LiftStepper_Stop();
    stop_requested = 0U;
    LiftStepper_ClearAbort();
    lift_depth_mm = 0.0f;
    error = MOTOR_CONTROL_ERROR_NONE;
    state = MOTOR_CONTROL_READY;
    return HAL_OK;
}

void MotorControl_Stop(void)
{
    stop_requested = 1U;
    LiftStepper_Abort();
    MG513_Stop();
    if (state != MOTOR_CONTROL_FAULT && state != MOTOR_CONTROL_DONE)
        state = MOTOR_CONTROL_STOPPED;
}

MotorControlState MotorControl_GetState(void) { return state; }
MotorControlError MotorControl_GetError(void) { return error; }
