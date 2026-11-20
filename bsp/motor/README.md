# 电机驱动使用说明

本文整理云台舵机、夹爪舵机、升降步进电机和 MG513 直流减速电机的函数接口、可调参数及调用示例。

## 自动搬运流程与标定

`Core/Src/motor_control.c` 使用阻塞式顺序流程，`Core/Inc/motor_control.h` 集中配置参数。`main.c` 完成外设初始化和 `MotorControl_Init()` 后进入按键等待：SW1(PB8)/SW2(PB9)/SW3(PB14)/SW4(PC13) 均为低电平按下、内部上拉、下降沿 EXTI，10 ms 去抖；`READY` 时任一按键启动一次 `MotorControl_RunOnce()`，`RUNNING` 期间任一按键通过 `MotorControl_RequestStop()` 请求中止。完成后保持 `DONE`，再次按键会先调用 `MotorControl_Rearm()` 再启动下一轮；中止后为 `STOPPED`，需复位才能再次启动。旧的 `MotorControl_Start()` / `MotorControl_Process()` 已移除。

启动前人工将底盘对齐取物台、升降机构置于最高处；云台不再需要人工摆位。初始化及3秒启动等待期间不下发云台位置、不启动云台PWM。自动流程为：张开夹爪并等待稳定 → 云台回到0°起点并等待稳定 → 下降取货 → 夹紧 → 抬升 → 前进270 mm → 停车静置200 ms → 云台旋转180° → 下降放货 → 松爪 → 升到最高处 → 返回起点 → 云台回到0° → 完成。只使用置物台2，不循环执行；结束后云台回到0°起点。

当前工作区 `MOTOR_CONTROL_CALIBRATED=1`，程序会执行自动流程。若要先锁定不运动，把它改回 `0`，程序会保持 `MOTOR_CONTROL_FAULT`，错误为 `MOTOR_CONTROL_ERROR_NOT_CALIBRATED`。以下参数必须实机标定后再置为1并重新编译：

| 配置宏（均在 motor_control.h） | 含义与填写规则 |
| --- | --- |
| `MOTOR_CONTROL_TRAVEL_SIGN` | 向置物区对应 MG513 正角度填 `1`，负角度填 `-1` |
| `MOTOR_CONTROL_LIFT_DOWN_FORWARD` | 步进正转下降填 `1`，反转下降填 `0` |
| `MOTOR_CONTROL_PICKUP_DEPTH_MM` | 从最高处下降到实际夹持位置的距离 D；当前示例50 mm |
| `MOTOR_CONTROL_LIFT_TRAVEL_MM` | 实测可用升降行程；当前示例100 mm，不能直接当作丝杆总长 |
| `MOTOR_CONTROL_GRIP_OPEN_DEG` | 实测张开角度0～180°；当前示例100° |
| `MOTOR_CONTROL_GRIP_CLOSED_DEG` | 实测夹紧角度0～180°，不能与张开角度相同；当前示例0° |
| `MOTOR_CONTROL_CARRY_LIFT_MM` | 取货后抬升量 H，默认10 mm，允许5～10 mm |

仍需标定 `gimbal_servo.h` 的云台端点和方向、`mg513.h` 的编码器计数及方向，并实机整定PID。夹爪“180°运动47 mm”不足以推算张开及夹紧角度。未填写参数或范围无效时，配置错误会锁存，不执行后续动作。

最高处为升降零点，向下为正：取货位置 D，运输位置 D−H，放货位置 D+40，结束位置0。启动检查 `D≥H` 和 `D+40≤可用行程`。从运输高度下降至放货高度的距离为 H+40，默认50 mm。丝杆导程8 mm/圈，对应1 mm为45°、100脉冲；步进驱动按脉冲取整，位置只是开环估计。

底盘编码器初始化位置为零点，此后启动前不要手动移动底盘。48 mm车轮直驱，270 mm约为输出轴644.58°；向置物区和返回起点分别下发绝对目标±644.58°和0°。夹爪等待初值1秒、云台等待初值1.5秒、每段行走超时15秒；参数均需结合负载验证。

| 控制接口 | 行为 |
| --- | --- |
| `MotorControl_Init()` | 初始化不运动；不能用来解除故障或重新归零 |
| `MotorControl_RunOnce()` | 仅在 `READY` 可调用；由按键触发，阻塞执行完整流程，成功返回 `HAL_OK`，失败返回 `HAL_ERROR`；DONE 后需先 Rearm |
| `MotorControl_Rearm()` | 仅在 `DONE` 后调用：确认无动作、清停止标志并回到 `READY`，供下一次 RunOnce |
| `MotorControl_RequestStop()` | EXTI 中调用，置停止请求并中止步进脉冲；`RUNNING` 时有效，流程在当前动作返回后结束 |
| `MotorControl_Stop()` | 主程序停机入口，停止底盘并锁定流程，不松爪、不关舵机PWM；运行中保持升降力矩，全部 DONE 后才失能；保留已有故障和完成状态 |
| `MotorControl_GetState()` | 返回未初始化、待命、运行、完成、停止或故障六种状态，不再区分每个动作 |
| `MotorControl_GetError()` | 返回无错误，或未标定、参数错误、驱动失败、行走超时四类错误 |

错误码只保存在内存，供调试器或调用者查询，没有串口发送或显示输出。驱动失败统一为 `MOTOR_CONTROL_ERROR_DRIVER`；去程和回程超时统一为 `MOTOR_CONTROL_ERROR_TIMEOUT`，保留未标定和配置错误两类保护。

所有控制接口只在主程序上下文调用，运行期间保持系统节拍和TIM4中断开启。启动、夹爪和云台旋转后的等待使用可中断的 1 ms 轮询；云台 `Home()` 内部的 1 s 等待不可中断；升降沿用阻塞驱动但支持中止请求；底盘等待循环每毫秒检查到位、停止请求和15秒超时。主程序阻塞时，TIM4中断仍继续执行PID。

主循环阻塞期间 EXTI 仍能记录按键并请求停止，因此按键不是异步急停：停止在当前动作返回或被中止后生效，最多需等云台 `Home()` 的 1 s 等待结束。升降没有限位反馈，舵机通过等待时间推定到位，软件无法确认失步、夹持成功或旋转完成。行走超时或驱动调用失败时停止底盘并立即返回故障，不再下发任何后续动作；保持已有夹持和升降力矩。

正常 `DONE` 后可按键重跑；若在中途 `STOPPED` 或 `FAULT`，必须复位并人工恢复起点、最高处和取物方向。先单轴标定，再空载检查流程和干涉，最后带载验收；软件模拟验证不替代实机验证。

### 云台人工初始位置接口

新增 `HAL_StatusTypeDef GimbalServo_ConfirmHome(void)`：在 `GimbalServo_Init()` 后、PWM从未启动时调用，仅记录人工已摆到起点，不写比较寄存器、不启动PWM。它没有传感器，不是自动归零。`GimbalServo_RotateCW180()` 随后首次启动PWM并转到终点。PWM启动失败时保留原状态及比较值；成功启动后不能再用 `ConfirmHome()` 重写位置。原有 `Home()` 接口仍可驱动云台回到起点。

GPIO初始化及CubeMX配置均将TB6600的低有效ENA设置为高电平禁用；升降动作完成后保持使能，全部流程 DONE 后才失能。Makefile已加入控制源文件，EIDE按现有Core目录扫描编译。

## 1. 接线与初始化条件

| 电机 | 型号或规格 | 控制信号 | STM32 引脚 |
| --- | --- | --- | --- |
| 云台舵机 | MG996R，180°版本 | PWM，TIM3_CH1（默认映射） | PA6 |
| 夹爪舵机 | MG996R，180°版本 | PWM，TIM3_CH2（默认映射） | PA7 |
| 升降步进电机 | 原步距1.8°，TB6600设为4细分 | PUL，TIM1_CH3 PWM | PA10 |
| 升降步进电机 | TB6600 | DIR，方向 | PB12 |
| 升降步进电机 | TB6600，低电平使能 | ENA，使能 | PB15 |
| MG513 直流电机 | 带编码器减速电机 | PWM，TIM1_CH1 | PA8 |
| MG513 直流电机 | 驱动方向 | dir1 / dir2 | PA4 / PA5 |
| MG513 直流电机 | 高电平使能 | enable | PA0 |
| MG513 直流电机 | 编码器 A / B | TIM2_CH1 / TIM2_CH2 | PA15 / PB3 |

两台舵机共用 TIM3。定时器时钟为72 MHz，预分频 `PSC=72-1`，周期 `ARR=19999`，两个通道均配置为 PWM1、高电平有效。此时 PWM 为50 Hz，每个计数对应1 μs。

步进电机使用 TIM1_CH3/PA10 硬件 PWM 发脉冲，TIM1 时基 1 MHz（`PSC=71`），运行期修改 ARR 调频，CCR3 固定 50% 占空；TIM1 更新中断负责计步和梯形加减速。驱动器细分需与 `LIFT_STEPPER_MICROSTEP` 一致，当前为 4 细分（800 步/圈）。

MG513 使用 TIM1_CH1 输出 10 kHz PWM（`PSC=0`、`ARR=7199`）；TIM4 产生 10 ms 控制中断（`PSC=72-1`、`ARR=10000-1`）；TIM2 以 TI12 四倍频读取 PA15/PB3 编码器，需要 TIM2 部分重映射 1，并关闭 JTAG（保留 SWD）释放 PA15/PB3。方向由 PA4/PA5 控制，PA0 为**高电平使能**。

> 注意：MG513 的 PWM（PA8/TIM1_CH1）和步进的脉冲（PA10/TIM1_CH3）共用 TIM1，但 PSC/ARR 需求不同，**不能同时运行**。每次 MG513 或步进开始动作前都会重新配置 TIM1；切换由 `Tim1Mux_*` 统一处理（声明在 `bsp/motor/inc/tim1_mux.h`，实现在 `bsp/motor/src/lift_stepper_motor.c`）。

工程需编译 `src` 中实际使用的 `.c` 文件（MG513 还需 `bsp/pid/pid.c`），并将对应 `inc` 目录加入头文件搜索路径。

## 2. 函数接口

```c
#include "gimbal_servo.h"
#include "gripper_servo.h"
#include "lift_stepper_motor.h"
#include "mg513.h"

/* 云台舵机 */
HAL_StatusTypeDef GimbalServo_Init(void);
HAL_StatusTypeDef GimbalServo_ConfirmHome(void);
HAL_StatusTypeDef GimbalServo_Home(void);
HAL_StatusTypeDef GimbalServo_RotateCW180(void);

/* 夹爪舵机 */
HAL_StatusTypeDef GripperServo_Init(void);
HAL_StatusTypeDef GripperServo_SetAngle(uint16_t angle);

/* 升降步进电机 */
HAL_StatusTypeDef LiftStepper_Init(void);
HAL_StatusTypeDef LiftStepper_Move(LiftStepperDirection direction, float angle_deg);
HAL_StatusTypeDef LiftStepper_SetSpeedRpm(float rpm);
float LiftStepper_GetSpeedRpm(void);
void LiftStepper_Abort(void);
void LiftStepper_ClearAbort(void);
void LiftStepper_Stop(void);
void LiftStepper_Disable(void);
uint8_t LiftStepper_IsBusy(void);

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
| `GimbalServo_Home()` | 下发云台起点位置并阻塞等待 `GIMBAL_SERVO_HOME_SETTLE_MS`；无位置反馈 |
| `GimbalServo_ConfirmHome()` | 人工已摆在起点时仅记录状态，不输出PWM；只允许在PWM从未启动时调用 |
| `GimbalServo_RotateCW180()` | 在回起点并等待后，或人工定位并调用ConfirmHome后，向顺时针终点转动；已在终点时重复调用返回错误，需先再次回起点 |
| `GripperServo_Init()` | 在 `MX_TIM3_Init()` 后初始化驱动，不启动 PWM、不下发位置 |
| `GripperServo_SetAngle(angle)` | 设置夹爪绝对目标角度，整数0～180°；下发后立即返回 |
| `LiftStepper_Init()` | 在系统时钟和 TIM1 初始化后调用；停止脉冲、驱动器失能，不产生运动 |
| `LiftStepper_Move(direction, angle_deg)` | 按方向相对转动指定角度，阻塞到全部脉冲发完；角度为有限的非负数，可超过360° |
| `LiftStepper_SetSpeedRpm(rpm)` | 设置后续动作的默认转速，单位 rpm，范围见头文件；运动中调用只影响下一次 |
| `LiftStepper_Abort()` | 请求中止当前/下一次动作；只置位，可在中断中调用 |
| `LiftStepper_ClearAbort()` | 清除中止请求，`MotorControl_RunOnce()` 启动时调用 |
| `LiftStepper_Stop()` | 立即停止脉冲，保持当前使能状态 |
| `LiftStepper_Disable()` | 立即停止脉冲并失能驱动器；`MotorControl` 只在全部 DONE 后调用 |
| `LiftStepper_IsBusy()` | 返回 1 表示正在发脉冲 |
| `MG513_Init()` | 启动 TIM2 编码器、TIM1 PWM、TIM4 10 ms 节拍；驱动保持失能，等待位置指令 |
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

舵机和步进电机没有位置反馈，返回成功不代表机械已到位。MG513 使用 TIM2 编码器做闭环反馈，位置指令为非阻塞下发，可通过 `MG513_MoveDone()` 判断动作状态。`MG513_MoveTo()` 相对上电零点，`MG513_MoveRelative()` 相对当前位置；两者都用“圈数+角度”统一表示。到位后驱动自动失能，不对外暴露单独的使能/速度接口。

### MG513 对外接口

MG513 对外只保留以下 6 个接口：

| 接口 | 说明 |
| --- | --- |
| `MG513_Init()` | 配置并启动 TIM2 编码器、TIM4 10 ms 中断；TIM1 PWM 在首次动作时接管，驱动保持失能 |
| `MG513_MoveTo(rounds, deg)` | 绝对位置：目标 = rounds×360° + deg；自动使能，立即返回 |
| `MG513_MoveRelative(rounds, deg)` | 相对位置：目标 = 当前角 + rounds×360° + deg；自动使能，立即返回 |
| `MG513_Stop()` | 立即清 PWM 并失能，目标改为当前位置 |
| `MG513_MoveDone()` | 返回 1 表示到位；到位的同一拍会自动失能 |
| `MG513_GetPosition()` | 返回累计位置 `{rounds, deg}`，总角度 = rounds×360° + deg |

使能控制、速度环、方向切换、PWM 限幅和到位判定均在 `mg513.c` 内部完成，不对外暴露。

### MG513 使能与定时器生命周期

| 时机 | PA0 使能 | TIM2 编码器 | TIM1 PWM | TIM4 10 ms |
| --- | --- | --- | --- | --- |
| `MX_GPIO_Init()` 后 | 低电平（失能） | 未启动 | 未启动 | 未启动 |
| `MG513_Init()` 成功 | 低电平（失能） | 启动，持续计数 | 未启动 | 启动，持续中断 |
| `MG513_MoveTo()` / `MG513_MoveRelative()` | 高电平（使能） | 保持运行 | 接管 TIM1，CH1 启动 | 保持运行 |
| 运动过程中 | 高电平（使能） | 保持运行 | CH1 保持运行 | 保持运行 |
| 到位自动判定 | 低电平（自动失能） | 保持运行 | CH1 保持运行，CCR=0 | 保持运行 |
| `MG513_Stop()` | 低电平（失能） | 保持运行 | CH1 保持运行，CCR=0 | 保持运行 |
| 步进开始动作 | 不参与 | 保持运行 | CH1 停止，切换 CH3 | 保持运行 |
| `MG513_Init()` 失败清理 | 低电平（失能） | 停止 | 未启动 | 不启动 |

定时器计数逻辑：

- TIM2 编码器在 `MG513_Init()` 中启动后不再停止；`motor_update()` 每 10 ms 读取一次 16 位 CNT，用有符号差扩展成 32 位累计计数。
- TIM1 由 MG513 和步进分时复用。MG513 在 `MoveTo/MoveRelative` 时通过 `Tim1Mux_TakeMg513()` 恢复 `PSC=0、ARR=7199` 并启动 CH1；步进在动作时通过 `Tim1Mux_TakeStepper()` 切到 `PSC=71`、动态 ARR 和 CH3。
- TIM1 更新中断只在步进运行时开启，用于计步和加减速；MG513 运行时关闭。
- TIM4 在 `MG513_Init()` 最后启动，10 ms 更新中断一直运行；`READY` 状态下中断只更新编码器和把 CCR 写 0，不跑 PID；只有 `RUNNING` 状态才执行串级 PID。
- `MG513_Stop()` 只停止电机输出并使能脚失能，不停止 TIM2/TIM4；TIM1 由下一次 MG513 或步进动作重新接管。
- `MG513_Init()` 会把 TIM4 ARR 强制为 `9999`；MG513 接管 TIM1 时再把 ARR 设为 `7199`。步进接管时按起始频率设置 ARR。

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
| `LIFT_STEPPER_MOTOR_STEP_ANGLE_DEG` | `1.8f` | 电机整步步距角 |
| `LIFT_STEPPER_MICROSTEP` | `4U` | 驱动器细分，需与 TB6600 拨码一致 |
| `LIFT_STEPPER_STEPS_PER_REV` | `800` | 每圈脉冲数 = 360° ÷ 1.8° × 4 |
| `LIFT_STEPPER_FORWARD_LEVEL` | `GPIO_PIN_SET` | 正转时 DIR 电平；方向相反时改为 `GPIO_PIN_RESET` |
| `LIFT_STEPPER_ENABLE_LEVEL` | `GPIO_PIN_RESET` | 使能电平，当前 TB6600 低电平使能 |
| `LIFT_STEPPER_DISABLE_LEVEL` | `GPIO_PIN_SET` | 失能电平，应与使能电平相反 |
| `LIFT_STEPPER_ENABLE_DELAY_MS` | `10U` | 使能后到第一个脉冲的等待时间，单位 ms |
| `LIFT_STEPPER_DEFAULT_SPEED_RPM` | `30.0f` | 默认转速，单位 rpm |
| `LIFT_STEPPER_MIN_SPEED_RPM` | `1.0f` | 允许的最小转速 |
| `LIFT_STEPPER_MAX_SPEED_RPM` | `120.0f` | 允许的最大转速 |
| `LIFT_STEPPER_START_FREQ_HZ` | `400U` | 加减速起始频率 |
| `LIFT_STEPPER_MAX_FREQ_HZ` | `20000U` | 脉冲频率上限，限制中断负载 |
| `LIFT_STEPPER_ACCEL_HZ_S` | `20000U` | 梯形加减速斜率，单位 Hz/s |
| `LIFT_STEPPER_TIMER_PSC` | `71U` | TIM1 步进模式预分频，72 MHz/(71+1) = 1 MHz |
| `LIFT_STEPPER_TIMER_HZ` | `1000000UL` | TIM1 步进时基 |

方向或使能极性接反时，优先改 `LIFT_STEPPER_FORWARD_LEVEL` / `LIFT_STEPPER_ENABLE_LEVEL`，并保证 `LIFT_STEPPER_DISABLE_LEVEL` 与使能电平相反。当前初始化会主动输出失能电平，不会在上电时自动转动。

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

步进脉冲由 TIM1_CH3 PWM 硬件产生，频率 `f = 1 MHz / (ARR + 1)`，CCR3 固定 50% 占空；TIM1 更新中断计步并执行梯形加减速。默认 30 rpm 对应 `30 ÷ 60 × 800 = 400 Hz`，50 mm（丝杆导程 8 mm）对应 5000 脉冲，约 12.5 秒。`LiftStepper_SetSpeedRpm()` 可在动作前修改默认转速。

`LiftStepper_Move()` 内部会重新接管 TIM1：停止 MG513 的 CH1、设置 `PSC=71`、按起始频率写入 ARR/CCR3，再启动 CH3 和更新中断。动作完成后停止 CH3 和更新中断，驱动器保持使能以维持升降位置；全部流程 DONE 后由 `LiftStepper_Disable()` 失能。MG513 下次动作时会重新接管 TIM1 并恢复 CH1 的 10 kHz 配置。

### MG513 直流电机：`inc/mg513.h`

| 参数 | 默认值 | 用途 |
| --- | --- | --- |
| `MG513_PWM_ARR` | `7199U` | TIM1 ARR，72 MHz/(7199+1) = 10 kHz |
| `MG513_PWM_MAX` | `7000` | PWM 计数绝对上限，需小于 ARR |
| `MG513_PWM_DEADBAND` | `50` | 小于此值的控制量按 0 处理，避免零附近换向 |
| `MG513_ENC_PPR` | `52.0f` | 电机轴一圈编码器计数（已含 TIM2 四倍频） |
| `MG513_GEAR_RATIO` | `30.0f` | 减速比，按实际电机修改 |
| `MG513_CNT_PER_OUTPUT_REV` | `52×30=1560` | 输出轴一圈计数，必须实测标定 |
| `MG513_SPEED_TAU_S` | `0.020f` | 速度反馈一阶低通时间常数，单位 s |
| `MG513_ENCODER_INVERT` | `1` | 编码器计数方向与电机正转相反时置 1 |
| `MG513_DIR_INVERT` | `0` | 方向接反时置 1 |
| `MG513_EN_ACTIVE_LOW` | `0` | 0=高电平使能（PA0）；低电平使能时改为 1 |
| `MG513_DONE_BAND_DEG` | `2.0f` | 到位角度容差，单位 ° |
| `MG513_DONE_SPEED_DEG_S` | `10.0f` | 到位速度阈值，单位 °/s |

`MG513_CNT_PER_OUTPUT_REV` 标定方法：手动把输出轴精确转一圈，读取 `MG513_GetPosition()` 并按 `实测角度 = rounds×360° + deg` 计算；若实测角度不是 360°，按 `新值 = 旧值 × 实测角度 ÷ 360°` 修正。不要只靠 `52×30` 估算。方向或编码器接反时优先改 `MG513_DIR_INVERT` / `MG513_ENCODER_INVERT`，不要改 PID 符号。

MG513 的使能完全由位置指令管理：`MG513_Init()` 后驱动保持失能；`MoveTo/MoveRelative` 自动使能；到位判定成立的同一拍清 PWM 并自动失能；`MG513_Stop()` 立即失能。

### PID 与串级控制：`bsp/pid/pid.h`

| 参数 | 默认值 | 用途 |
| --- | --- | --- |
| `PID_CTRL_DT_S` | `0.010f` | 内环周期，单位 s，必须与 TIM2 的 10 ms 一致 |
| `PID_POS_DIV` | `5U` | 外环分频，外环周期 = 10 ms × 5 = 50 ms |
| `PID_SPEED_MAX_DEG_S` | `120.0f` | 内环给定钳位，单位 °/s（20 rpm） |
| `PID_SPEED_RAMP_DEG_S2` | `1440.0f` | 内部速度模式斜坡加速度，单位 °/s²；MG513 不对外提供速度指令 |
| `PID_OUT_KP/KI/KD` | `4.0/1.8/0.06` | 位置环参数，输入 °，输出 °/s |
| `PID_OUT_INT_BAND_DEG` | `50.0f` | 位置环积分分离带，单位 ° |
| `PID_IN_KP/KI/KD` | `15.0/2.5/0.06` | 速度环参数，输入 °/s，输出 PWM 计数 |
| `PID_IN_INT_BAND_DEG_S` | `180.0f` | 速度环积分分离带，单位 °/s |

PID 参数只是可运行初值，必须按实机先整定内环（速度环）再整定外环（位置环）。`PID_CTRL_DT_S` 必须与 TIM4 中断周期一致，否则速度/积分换算会出错。

## 4. 调用示例

在 `main.c` 中加入对应头文件。完成 `HAL_Init()`、系统时钟配置和 `MX_GPIO_Init()` 后，先初始化相关定时器：舵机需要 `MX_TIM3_Init()`，MG513 需要 `MX_TIM1_Init()`、`MX_TIM2_Init()` 和 `MX_TIM4_Init()`，再初始化驱动：

```c
if (GimbalServo_Init() != HAL_OK) { Error_Handler(); }
if (GripperServo_Init() != HAL_OK) { Error_Handler(); }
if (LiftStepper_Init() != HAL_OK) { Error_Handler(); }
if (MG513_Init() != HAL_OK) { Error_Handler(); }
```

`MG513_Init()` 会启动 TIM2 编码器和 TIM4 10 ms 中断，但驱动保持失能，TIM1 PWM 留到首次动作时接管；首次调用 `MG513_MoveTo()` / `MG513_MoveRelative()` 时自动使能，到位后自动失能。

按业务需求调用动作，下面仅为示例，不需要放入无限循环中反复执行：

```c
/* 云台：Home 内部已等待回位，再顺时针转180°。 */
if (GimbalServo_Home() != HAL_OK) { Error_Handler(); }
if (GimbalServo_RotateCW180() != HAL_OK) { Error_Handler(); }

/* 夹爪：目标角度仅作示例，实际开合角度待调试。 */
if (GripperServo_SetAngle(90) != HAL_OK) { Error_Handler(); }

/* 步进：设置 30 rpm，正转180°，再反转90°；每次都等待脉冲发完再继续。 */
if (LiftStepper_SetSpeedRpm(30.0f) != HAL_OK) { Error_Handler(); }
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
    /* 等待到位；控制仍在 TIM4 中断中运行，主循环可做其他非阻塞任务。 */
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

MG513 的位置接口只负责下发目标，立即返回，实际运动由 TIM4 中断中的串级 PID 完成；到位后自动清 PWM 并失能。`MG513_MoveDone()` 只是轮询判断，不会阻塞；示例中的空 `while` 仅用于演示等待，实际工程建议在主循环中非阻塞轮询。速度环仅在内部运行，不对外提供速度控制接口。

步进函数只在主循环等普通程序上下文调用，不在中断中调用。`HAL_Delay()` 依赖系统节拍，运行期间不要关闭或暂停节拍中断。阻塞运行期间主程序不能继续处理其他任务，已有的舵机硬件 PWM 仍会继续输出。

如果需要将厘米换算为步进角度，先确定传动机构每转移动的距离：`角度 = 移动距离 ÷ 每转移动距离 × 360°`，两个距离使用相同单位。
bsp/motor/README.md
