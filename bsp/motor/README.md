# 电机驱动使用说明

本文整理云台舵机、夹爪舵机、升降步进电机和 MG513 直流减速电机的函数接口、可调参数及调用示例。

## 1. 接线与初始化条件

| 电机 | 型号或规格 | 控制信号 | STM32 引脚 |
| --- | --- | --- | --- |
| 云台舵机 | MG996R，180°版本 | PWM，TIM3_CH1 | PA6 |
| 夹爪舵机 | MG996R，180°版本 | PWM，TIM3_CH2 | PA7 |
| 升降步进电机 | 原步距1.8°，TB6600设为4细分 | PUL，脉冲 | PA10 |
| 升降步进电机 | TB6600 | DIR，方向 | PB12 |
| 升降步进电机 | TB6600，低电平使能 | ENA，使能 | PB15 |
| MG513 直流电机 | 带编码器减速电机 | PWM，TIM4_CH1 | PB6 |
| MG513 直流电机 | 驱动方向 | dir1 / dir2 | PB0 / PB1 |
| MG513 直流电机 | 高电平使能 | enable | PB5 |
| MG513 直流电机 | 编码器 A / B | TIM1_CH1 / TIM1_CH2 | PA8 / PA9 |

两台舵机共用 TIM3。定时器时钟为72 MHz，预分频 `PSC=72-1`，周期 `ARR=19999`，两个通道均配置为 PWM1、高电平有效。此时 PWM 为50 Hz，每个计数对应1 μs。

步进电机通过 GPIO 和 `HAL_Delay()` 产生脉冲，不使用 DWT，也不占用 TIM3。驱动器的细分开关需实际设为4细分。

MG513 使用 TIM4_CH1 输出 10 kHz PWM（`PSC=0`、`ARR=7199`）；TIM2 产生 10 ms 控制中断（`PSC=72-1`、`ARR=10000-1`）；TIM1 以 TI12 四倍频读取 PA8/PA9 编码器。方向由 PB0/PB1 控制，PB5 为**高电平使能**。

工程需编译 `src` 中实际使用的 `.c` 文件（MG513 还需 `bsp/pid/pid.c`），并将对应 `inc` 目录加入头文件搜索路径。

## 2. 函数接口

```c
#include "gimbal_servo.h"
#include "gripper_servo.h"
#include "lift_stepper_motor.h"
#include "mg513.h"

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

/* MG513 直流减速电机 */
typedef struct {
    int32_t rounds; /* 整圈数 */
    float deg;      /* 剩余角度，可正负 */
} mg513_position_t;

HAL_StatusTypeDef MG513_Init(void);
HAL_StatusTypeDef MG513_MoveTo(int32_t rounds, float deg);
HAL_StatusTypeDef MG513_MoveRelative(int32_t rounds, float deg);
void MG513_Stop(void);
uint8_t MG513_MoveDone(void);
mg513_position_t MG513_GetPosition(void);
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
| `MG513_Init()` | 启动 TIM1 编码器、TIM4 PWM、TIM2 10 ms 节拍；驱动保持失能，等待位置指令 |
| `MG513_MoveTo(rounds, deg)` | 位置模式，转到绝对目标角度（圈数×360°+deg）；自动使能，立即返回 |
| `MG513_MoveRelative(rounds, deg)` | 位置模式，相对当前位置转动（圈数×360°+deg）；自动使能，立即返回 |
| `MG513_Stop()` | 立即停止：清 PWM、失能驱动，目标设为当前位置 |
| `MG513_MoveDone()` | 返回 1 表示已到位；到位的同一拍会自动清 PWM 并失能驱动 |
| `MG513_GetPosition()` | 返回累计位置结构体 `{rounds, deg}`，总角度 = rounds×360° + deg |

步进方向参数：

```c
LIFT_STEPPER_FORWARD // 正转，枚举值0
LIFT_STEPPER_REVERSE // 反转，枚举值1
```

返回值说明：

| 返回值 | 含义 |
| --- | --- |
| `HAL_OK` | 指令成功：舵机已下发目标、步进已发完脉冲，或 MG513 已接受目标 |
| `HAL_ERROR` | 未初始化、参数不合法、云台调用顺序错误或硬件配置不匹配等 |
| `HAL_BUSY` | 步进电机正在执行动作；舵机底层启动失败时也会传回相应 HAL 状态 |

舵机和步进电机没有位置反馈，返回成功不代表机械已到位。MG513 使用 TIM1 编码器做闭环反馈，位置指令为非阻塞下发，可通过 `MG513_MoveDone()` 判断动作状态。`MG513_MoveTo()` 相对上电零点，`MG513_MoveRelative()` 相对当前位置；两者都用“圈数+角度”统一表示。到位后驱动自动失能，不对外暴露单独的使能/速度接口。

### MG513 对外接口

MG513 对外只保留以下 6 个接口：

| 接口 | 说明 |
| --- | --- |
| `MG513_Init()` | 配置并启动 TIM1 编码器、TIM4 PWM、TIM2 10 ms 中断；驱动保持失能 |
| `MG513_MoveTo(rounds, deg)` | 绝对位置：目标 = rounds×360° + deg；自动使能，立即返回 |
| `MG513_MoveRelative(rounds, deg)` | 相对位置：目标 = 当前角 + rounds×360° + deg；自动使能，立即返回 |
| `MG513_Stop()` | 立即清 PWM 并失能，目标改为当前位置 |
| `MG513_MoveDone()` | 返回 1 表示到位；到位的同一拍会自动失能 |
| `MG513_GetPosition()` | 返回累计位置 `{rounds, deg}`，总角度 = rounds×360° + deg |

使能控制、速度环、方向切换、PWM 限幅和到位判定均在 `mg513.c` 内部完成，不对外暴露。

### MG513 使能与定时器生命周期

| 时机 | PB5 使能 | TIM1 编码器 | TIM4 PWM | TIM2 10 ms |
| --- | --- | --- | --- | --- |
| `MX_GPIO_Init()` 后 | 低电平（失能） | 未启动 | 未启动 | 未启动 |
| `MG513_Init()` 成功 | 低电平（失能） | 启动，持续计数 | 启动，CCR=0 | 启动，持续中断 |
| `MG513_MoveTo()` / `MG513_MoveRelative()` | 高电平（使能） | 保持运行 | 保持运行 | 保持运行 |
| 运动过程中 | 高电平（使能） | 保持运行 | 保持运行 | 保持运行 |
| 到位自动判定 | 低电平（自动失能） | 保持运行 | 保持运行，CCR=0 | 保持运行 |
| `MG513_Stop()` | 低电平（失能） | 保持运行 | 保持运行，CCR=0 | 保持运行 |
| `MG513_Init()` 失败清理 | 低电平（失能） | 停止 | 停止 | 不启动 |

定时器计数逻辑：

- TIM1 编码器在 `MG513_Init()` 中启动后不再停止；`motor_update()` 每 10 ms 读取一次 16 位 CNT，用有符号差扩展成 32 位累计计数。
- TIM4 PWM 在 `MG513_Init()` 中启动后持续输出；失能或空闲时只把 CCR 写 0，不调用 `HAL_TIM_PWM_Stop()`。
- TIM2 在 `MG513_Init()` 最后启动，10 ms 更新中断一直运行；`READY` 状态下中断只更新编码器和把 CCR 写 0，不跑 PID；只有 `RUNNING` 状态才执行串级 PID。
- `MG513_Stop()` 只停止电机输出并使能脚失能，不会停止或暂停 TIM1/TIM2/TIM4。运行期没有对外提供定时器停止/暂停接口。
- `MG513_Init()` 会把 TIM2 ARR 强制为 `9999`、TIM4 ARR 强制为 `7199`，防止 CubeMX 重新生成后节拍或 PWM 频率改变。

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

### MG513 直流电机：`inc/mg513.h`

| 参数 | 默认值 | 用途 |
| --- | --- | --- |
| `MG513_PWM_ARR` | `7199U` | TIM4 ARR，72 MHz/(7199+1) = 10 kHz |
| `MG513_PWM_MAX` | `7000` | PWM 计数绝对上限，需小于 ARR |
| `MG513_PWM_DEADBAND` | `50` | 小于此值的控制量按 0 处理，避免零附近换向 |
| `MG513_ENC_PPR` | `52.0f` | 电机轴一圈编码器计数（已含 TIM1 四倍频） |
| `MG513_GEAR_RATIO` | `30.0f` | 减速比，按实际电机修改 |
| `MG513_CNT_PER_OUTPUT_REV` | `52×30=1560` | 输出轴一圈计数，必须实测标定 |
| `MG513_SPEED_TAU_S` | `0.020f` | 速度反馈一阶低通时间常数，单位 s |
| `MG513_ENCODER_INVERT` | `1` | 编码器计数方向与电机正转相反时置 1 |
| `MG513_DIR_INVERT` | `0` | 方向接反时置 1 |
| `MG513_EN_ACTIVE_LOW` | `0` | 0=高电平使能（PB5）；低电平使能时改为 1 |
| `MG513_DONE_BAND_DEG` | `2.0f` | 到位角度容差，单位 ° |
| `MG513_DONE_SPEED_DEG_S` | `10.0f` | 到位速度阈值，单位 °/s |

`MG513_CNT_PER_OUTPUT_REV` 标定方法：手动把输出轴精确转一圈，读取 `MG513_GetPosition()` 并按 `实测角度 = rounds×360° + deg` 计算；若实测角度不是 360°，按 `新值 = 旧值 × 实测角度 ÷ 360°` 修正。不要只靠 `52×30` 估算。方向或编码器接反时优先改 `MG513_DIR_INVERT` / `MG513_ENCODER_INVERT`，不要改 PID 符号。

MG513 的使能完全由位置指令管理：`MG513_Init()` 后驱动保持失能；`MoveTo/MoveRelative` 自动使能；到位判定成立的同一拍清 PWM 并自动失能；`MG513_Stop()` 立即失能。

### PID 与串级控制：`bsp/pid/pid.h`

| 参数 | 默认值 | 用途 |
| --- | --- | --- |
| `PID_CTRL_DT_S` | `0.010f` | 内环周期，单位 s，必须与 TIM2 的 10 ms 一致 |
| `PID_POS_DIV` | `5U` | 外环分频，外环周期 = 10 ms × 5 = 50 ms |
| `PID_SPEED_MAX_DEG_S` | `720.0f` | 内环给定钳位，单位 °/s |
| `PID_SPEED_RAMP_DEG_S2` | `1440.0f` | 内部速度模式斜坡加速度，单位 °/s²；MG513 不对外提供速度指令 |
| `PID_OUT_KP/KI/KD` | `4.0/1.8/0.06` | 位置环参数，输入 °，输出 °/s |
| `PID_OUT_INT_BAND_DEG` | `50.0f` | 位置环积分分离带，单位 ° |
| `PID_IN_KP/KI/KD` | `15.0/2.5/0.06` | 速度环参数，输入 °/s，输出 PWM 计数 |
| `PID_IN_INT_BAND_DEG_S` | `180.0f` | 速度环积分分离带，单位 °/s |

PID 参数只是可运行初值，必须按实机先整定内环（速度环）再整定外环（位置环）。`PID_CTRL_DT_S` 必须与 TIM2 中断周期一致，否则速度/积分换算会出错。

## 4. 调用示例

在 `main.c` 中加入对应头文件。完成 `HAL_Init()`、系统时钟配置和 `MX_GPIO_Init()` 后，先初始化相关定时器：舵机需要 `MX_TIM3_Init()`，MG513 需要 `MX_TIM1_Init()`、`MX_TIM2_Init()` 和 `MX_TIM4_Init()`，再初始化驱动：

```c
if (GimbalServo_Init() != HAL_OK) { Error_Handler(); }
if (GripperServo_Init() != HAL_OK) { Error_Handler(); }
if (LiftStepper_Init() != HAL_OK) { Error_Handler(); }
if (MG513_Init() != HAL_OK) { Error_Handler(); }
```

`MG513_Init()` 会自行启动编码器、PWM 和 TIM2 10 ms 中断，但驱动保持失能；首次调用 `MG513_MoveTo()` / `MG513_MoveRelative()` 时自动使能，到位后自动失能。

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

/* MG513：转到绝对 2 圈 + 90°，再相对转 0 圈 + 45°。 */
if (MG513_MoveTo(2, 90.0f) != HAL_OK) { Error_Handler(); }
while (MG513_MoveDone() == 0U)
{
    /* 等待到位；控制仍在 TIM2 中断中运行，主循环可做其他非阻塞任务。 */
}

if (MG513_MoveRelative(0, 45.0f) != HAL_OK) { Error_Handler(); }
while (MG513_MoveDone() == 0U)
{
}

/* 查询累计位置：rounds=2, deg=135 表示 855°。 */
mg513_position_t pos = MG513_GetPosition();

/* 停止：立即清 PWM 并失能。 */
MG513_Stop();
```

MG513 的位置接口只负责下发目标，立即返回，实际运动由 TIM2 中断中的串级 PID 完成；到位后自动清 PWM 并失能。`MG513_MoveDone()` 只是轮询判断，不会阻塞；示例中的空 `while` 仅用于演示等待，实际工程建议在主循环中非阻塞轮询。速度环仅在内部运行，不对外提供速度控制接口。

步进函数只在主循环等普通程序上下文调用，不在中断中调用。`HAL_Delay()` 依赖系统节拍，运行期间不要关闭或暂停节拍中断。阻塞运行期间主程序不能继续处理其他任务，已有的舵机硬件 PWM 仍会继续输出。

如果需要将厘米换算为步进角度，先确定传动机构每转移动的距离：`角度 = 移动距离 ÷ 每转移动距离 × 360°`，两个距离使用相同单位。
bsp/motor/README.md