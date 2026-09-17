# 设计说明

## 1. 系统目标

Project25 是一个小型单轴闭环运动控制节点，使用 STM32F103、N20 有刷减速电机、AB 增量式 Encoder 与 DRV8833。项目聚焦单轴闭环运动控制核心链路，通过实机验证反馈、控制、轨迹、状态管理、Homing 与安全边界的协同关系。

## 2. 软件架构

`Motion_Update()` 是唯一的 Motor 输出入口：它读取 Encoder 反馈、按当前状态调度控制流程、调用 Controller，并写入带方向的 PWM。未来的通信模块只能提交命令，不能直接写 PWM。

```text
Position Controller
        |
        v
Trajectory / Acceleration Limit
        |
        v
Speed PI + Feedforward
        |
        v
Motor Driver
        |
        v
N20 Motor + Encoder Feedback
```

当前只使用一个周期性 FreeRTOS 任务：`freertos.c` 每 20 ms 调度一次 `Motion_Update()`；`motion.c` 负责运动流程；`controller.c` 保存控制计算及其私有状态。

当前装配状态下，以约 `2150 PWM / 42 RPM` 作为低速 Feedforward 标定边界点，低于该区间采用平滑插值，避免将中高速标定关系直接外推到零速附近。

## 3. Motion State Machine

```text
INIT -> IDLE -> READY
                  |  \
                  |   \ position command
                  |    -> RUN_POSITION -> READY
                  |
                  +---- home command -> HOMING -> READY
                                           |
                                           +-- timeout -> FAULT
```

`FAULT` 会强制 PWM 为零，且不会自动恢复之前的命令。源码中保留 `RUN_SPEED` 枚举状态，但当前尚未形成独立速度命令流程；进入该状态时输出为零。

当前位置与 Homing 命令仍是本地验证标志，尚不是通信命令接口。

## 4. Homing 设计

增量式 Encoder 上电后的计数零点不等于机械零点。当前软件验证流程以 `home_rpm` 驱动电机，采样 PA0 上的 `HOME_SIM`；触发后关闭 PWM、清零 TIM4 与软件位置计数、Reset Speed PI / Trajectory，并返回 `READY`。

若 10 秒内未触发 `HOME_SIM`，系统进入 `FAULT` 并关闭 PWM。这证明了软件状态机流程；Mechanical Homing 重复性、开关消抖、真实 KW11 安装与 Hardware Limit 仍未验证。

## 5. Limit / Fault 设计

当前代码已经包含 Homing Timeout -> `FAULT` 的最小故障路径。Software Position Limit 与 Hardware Limit 是下一实现阶段，当前不宣称已实现或已上板验证。

后续实现遵守以下规则：

- 只有 motion core 可以控制 PWM；
- 发生 Fault 必须停止 PWM；
- Fault Reset 不得自动继续旧命令；
- Encoder 清零本身不能作为机械零点依据；
- 通信模块可以请求运动，不能直接驱动电机。

## 6. 设计边界

项目不为展示层次而增加多个 Task、Manager 或空模块。当前 Motor 与 Encoder 的 HAL 操作量较小，继续由 motion core 管理；进一步拆分不会提升当前代码的可读性或复用价值。

## 7. 近期收口

- Software Position Limit
- Basic Fault Handling / Fault Reset
- 真实 KW11 Mechanical Homing
- Homing Repeatability
- Hardware Limit

## 8. 可选扩展

- 轻量 Command Interface
- Modbus RTU
- Parameter Persistence
- IWDG / Task Monitor
- 更完整的 Fault Reporting
