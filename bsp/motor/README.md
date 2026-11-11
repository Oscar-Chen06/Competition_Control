# 电机驱动使用说明

本文整理云台舵机、夹爪舵机和升降步进电机的函数接口、可调参数及调用示例。

## 1. 接线与初始化条件

| 电机 | 型号或规格 | 控制信号 | STM32 引脚 |
| --- | --- | --- | --- |
| 云台舵机 | MG996R，180°版本 | PWM，TIM3_CH1 | PA6 |
| 夹爪舵机 | MG996R，180°版本 | PWM，TIM3_CH2 | PA7 |
| 升降步进电机 | 原步距1.8°，TB6600设为4细分 | PUL，脉冲 | PA10 |
| 升降步进电机 | TB6600 | DIR，方向 | PB12 |
| 升降步进电机 | TB6600，低电平使能 | ENA，使能 | PB15 |

两台舵机共用 TIM3。定时器时钟为72 MHz，预分频 `PSC=72-1`，周期 `ARR=19999`，两个通道均配置为 PWM1、高电平有效。此时 PWM 为50 Hz，每个计数对应1 μs。

步进电机通过 GPIO 和 `HAL_Delay()` 产生脉冲，不使用 DWT，也不占用 TIM3。驱动器的细分开关需实际设为4细分。

工程需编译 `src` 中对应的三个 `.c` 文件，并将 `inc` 加入头文件搜索目录。

## 2. 函数接口

```c
#include "gimbal_servo.h"
#include "gripper_servo.h"
#include "lift_stepper_motor.h"

/* 云台舵机 */
HAL_StatusTypeDef GimbalServo_Init(void);
HAL_StatusTypeDef GimbalServo_Home(void);
HAL_StatusTypeDef GimbalServo_RotateCW180(void);

/* 夹爪舵机 */
HAL_StatusTypeDef GripperServo_Init(void);
HAL_StatusTypeDef GripperServo_SetAngle(uint16_t angle);

/* 升降步进电机 */
HAL_StatusTypeDef LiftStepper_Init(void);
HAL_StatusTypeDef LiftStepper_Move(LiftStepperDirection direction, float angle_deg);
```

| 接口 | 作用与参数 |
| --- | --- |
| `GimbalServo_Init()` | 在 `MX_TIM3_Init()` 后初始化驱动，不启动 PWM、不下发位置 |
| `GimbalServo_Home()` | 下发云台起点位置，立即返回；调用方需等待舵机到位 |
| `GimbalServo_RotateCW180()` | 在回起点并等待后，向顺时针终点转动；已在终点时重复调用返回错误，需先再次回起点 |
| `GripperServo_Init()` | 在 `MX_TIM3_Init()` 后初始化驱动，不启动 PWM、不下发位置 |
| `GripperServo_SetAngle(angle)` | 设置夹爪绝对目标角度，整数0～180°；下发后立即返回 |
| `LiftStepper_Init()` | 在系统时钟和 GPIO 初始化后调用；脉冲拉低、驱动器禁用，不产生运动 |
| `LiftStepper_Move(direction, angle_deg)` | 按方向相对转动指定角度，阻塞到全部脉冲发完；角度为有限的非负数，可超过360° |

步进方向参数：

```c
LIFT_STEPPER_FORWARD // 正转，枚举值0
LIFT_STEPPER_REVERSE // 反转，枚举值1
```

返回值说明：

| 返回值 | 含义 |
| --- | --- |
| `HAL_OK` | 指令成功：舵机已下发目标，或步进已发完脉冲 |
| `HAL_ERROR` | 未初始化、参数不合法、云台调用顺序错误或硬件配置不匹配等 |
| `HAL_BUSY` | 步进电机正在执行动作；舵机底层启动失败时也会传回相应 HAL 状态 |

这些驱动没有位置反馈。返回成功不代表传感器已确认机械到位。舵机下发目标后持续输出 PWM 保持位置；步进运动结束后维持使能和保持力矩。

## 3. 可调参数

以下宏在对应头文件中修改，修改后需重新编译工程。

### 云台：`inc/gimbal_servo.h`

| 参数 | 默认值 | 用途 |
| --- | --- | --- |
| `GIMBAL_SERVO_MIN_PULSE_US` | `500U` | 最小脉宽，单位μs，用于校准一端位置 |
| `GIMBAL_SERVO_MAX_PULSE_US` | `2500U` | 最大脉宽，单位μs，用于校准另一端位置 |
| `GIMBAL_SERVO_CW_DECREASES_PULSE` | `1U` | 默认脉宽减小对应顺时针；实机方向相反时改为 `0U` |
| `GIMBAL_SERVO_HOME_SETTLE_MS` | `1000U` | 回起点后建议等待时间，单位ms，需在调用处传给 `HAL_Delay()` |

顺时针以正对舵机输出轴观察为准。默认起点为2500 μs、顺时针终点为500 μs。端点脉宽和实际180°行程需要上机校准；修改方向参数后，两端交换。

### 夹爪：`inc/gripper_servo.h`

| 参数 | 默认值 | 用途 |
| --- | --- | --- |
| `GRIPPER_SERVO_MIN_PULSE_US` | `500U` | 0°对应脉宽，单位μs |
| `GRIPPER_SERVO_MAX_PULSE_US` | `2500U` | 180°对应脉宽，单位μs |
| `GRIPPER_SERVO_MAX_ANGLE_DEG` | `180U` | 角度换算量程，180°版本保持此值 |

默认线性换算为：0°→500 μs、90°→1500 μs、180°→2500 μs。实际夹爪开合角度通过 `GripperServo_SetAngle()` 的参数调整，待机械调试后确定。

`GRIPPER_SERVO_MAX_ANGLE_DEG` 同时参与脉宽换算，改变它会改变整个角度比例，不能仅将它作为机械限位使用。

### 步进：`inc/lift_stepper_motor.h`

| 参数 | 默认值 | 用途 |
| --- | --- | --- |
| `LIFT_STEPPER_PULSES_PER_REV` | `800U` | 每圈脉冲数，必须与电机步距及驱动器实际细分一致 |
| `LIFT_STEPPER_FORWARD_LEVEL` | `GPIO_PIN_SET` | 正转时 DIR 的电平；方向相反时改为 `GPIO_PIN_RESET` |
| `LIFT_STEPPER_ENABLE_LEVEL` | `GPIO_PIN_RESET` | 当前 TB6600 的使能电平，保持低电平有效配置 |
| `LIFT_STEPPER_PULSE_DELAY_MS` | `1U` | 高、低电平各自的延时时间，单位ms；增大可降低速度，使用正整数 |
| `LIFT_STEPPER_ENABLE_DELAY_MS` | `10U` | 设置方向和使能后，到第一个脉冲前的等待时间，单位ms |

当前初始化时 ENA 固定拉高禁用，与已确认的低电平使能配套；若更换为相反极性的驱动器，不能只修改使能宏，还需同步调整初始化电平。

角度与脉冲换算：

```text
每圈脉冲数 = 360° ÷ 1.8° × 4 = 800
单脉冲角度 = 360° ÷ 800 = 0.45°
动作脉冲数 = 四舍五入(角度 × 800 ÷ 360)
```

| 指令角度 | 脉冲数 |
| --- | --- |
| 90° | 200 |
| 180° | 400 |
| 360° | 800 |

零脉冲动作不改变输出。角度换算后超出32位脉冲计数范围时返回错误。

当前不进行加减速。默认每个脉冲高、低电平各调用一次 `HAL_Delay(1)`；理想2 ms周期对应500脉冲/秒、37.5转/分，但 HAL 的节拍取整和中断会使实际周期更长，不能将该值当作精确转速。

## 4. 调用示例

在 `main.c` 中加入三个头文件。完成 `HAL_Init()`、系统时钟配置、`MX_GPIO_Init()` 和 `MX_TIM3_Init()` 后，再初始化驱动：

```c
if (GimbalServo_Init() != HAL_OK) { Error_Handler(); }
if (GripperServo_Init() != HAL_OK) { Error_Handler(); }
if (LiftStepper_Init() != HAL_OK) { Error_Handler(); }
```

按业务需求调用动作，下面仅为示例，不需要放入无限循环中反复执行：

```c
/* 云台：先回起点，等待后顺时针转180°。 */
if (GimbalServo_Home() != HAL_OK) { Error_Handler(); }
HAL_Delay(GIMBAL_SERVO_HOME_SETTLE_MS);
if (GimbalServo_RotateCW180() != HAL_OK) { Error_Handler(); }

/* 夹爪：目标角度仅作示例，实际开合角度待调试。 */
if (GripperServo_SetAngle(90) != HAL_OK) { Error_Handler(); }

/* 步进：正转180°，再反转90°；每次都等待脉冲发完再继续。 */
if (LiftStepper_Move(LIFT_STEPPER_FORWARD, 180.0f) != HAL_OK)
{
    Error_Handler();
}
if (LiftStepper_Move(LIFT_STEPPER_REVERSE, 90.0f) != HAL_OK)
{
    Error_Handler();
}
```

步进函数只在主循环等普通程序上下文调用，不在中断中调用。`HAL_Delay()` 依赖系统节拍，运行期间不要关闭或暂停节拍中断。阻塞运行期间主程序不能继续处理其他任务，已有的舵机硬件 PWM 仍会继续输出。

如果需要将厘米换算为步进角度，先确定传动机构每转移动的距离：`角度 = 移动距离 ÷ 每转移动距离 × 360°`，两个距离使用相同单位。
