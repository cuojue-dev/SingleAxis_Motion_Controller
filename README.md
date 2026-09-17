# Project25 单轴闭环运动控制节点

## 1. 项目简介

基于 STM32F103、FreeRTOS、N20 有刷减速电机、AB 增量式 Encoder 与 DRV8833 的单轴闭环运动控制项目。

当前版本聚焦一条完整且可阅读的控制链路：Encoder 反馈、Speed Control、相对位置控制、Acceleration-Limited Trajectory、Motion State Machine，以及初步的 Homing / Fault 流程。

## 2. 当前状态

**Work in Progress**

### 已上板观察验证

- Feedforward、Speed P 与 Speed PI
- Integral Limit 与 Conditional Anti-Windup
- `+80 RPM` / `-80 RPM` 双向速度闭环
- Stop Response 与轻微手动负载扰动恢复
- Position P -> Trajectory -> Speed PI 串级控制
- Acceleration-Limited Trajectory：最大 `60 RPM`，最大加速度 `100 RPM/s`
- `±700`、`±1400`、`±2800 Count` 相对定位；最终均进入配置的 `±100 Count` 容差
- `HOME_SIM` Homing 软件状态机，以及 Homing 10 秒超时 -> `FAULT`

### 下一步

- Software Position Limit
- Basic Fault Handling / Fault Reset

### 尚未完成真实机械验证

- KW11 Mechanical Homing
- Homing Repeatability
- Hardware Limit
- Final Mechanical Zero Reference

`HOME_SIM` 是用于验证软件状态机路径的板载输入，不代表最终机械 Homing 的重复精度。

## 3. 控制架构

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

`motion.c` 是唯一允许写入 PWM 的模块。`controller.c` 负责 Speed PI、Position Controller、Feedforward、Integral Limit、Conditional Anti-Windup 与 Trajectory 计算；FreeRTOS 任务每 100 ms 调用一次 `Motion_Update()`。

## 4. Motion State Machine

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

进入 `FAULT` 后 PWM 被关闭，系统不会自动继续故障前的命令。当前 Homing 使用 `HOME_SIM` 验证状态机；真实限位开关行为仍待验证。

## 5. 实机验证

| 证据 | 支持的结论 |
| --- | --- |
| ![Speed PI 响应](docs/images/speed_pi_response.png) | Speed PI 闭环响应 |
| ![Anti-Windup 响应](docs/images/speed_anti_windup.png) | Conditional Anti-Windup 行为 |
| ![轨迹速度响应](docs/images/cascade_trajectory_speed_response.png) | 串级控制中的 Acceleration-Limited Trajectory |
| ![位置响应](docs/images/position_response_initial.png) | `+2800 Count` 初步响应；过冲修正后进入容差 |

`speed_pi_restart_overshoot_pre_anti_windup.png` 保留在 `docs/images`，用于工程过程对照，不作为 README 主验证图。

## 6. 硬件 / 软件环境

- MCU：STM32F103ZET6
- RTOS：FreeRTOS + CMSIS-RTOS2
- Motor Driver：DRV8833，TIM3 PWM（PA6 / PA7）
- Encoder：TIM4 Encoder Mode（PD12 / PD13）
- Homing 模拟输入：PA0（`HOME_SIM`）
- 调试串口：USART1，115200 bps
- 工具：STM32CubeMX、STM32CubeIDE、Git

## 7. 仓库结构

```text
Firmware/
  Core/Src/freertos.c      FreeRTOS 任务创建与周期调度
  Core/Src/motion.c        状态机、Homing、Encoder/PWM 运动流程
  Core/Src/controller.c    Feedforward、PI、Position P、Trajectory
  Core/Inc/                对应头文件与 CubeMX 生成头文件
docs/
  design.md                设计说明与当前边界
  images/                  已筛选的验证证据
```

## 8. 当前限制与后续计划

Project25 当前仍以完成运动控制节点核心闭环为目标。

### 近期收口

- Software Position Limit
- Basic Fault Handling / Fault Reset
- 真实 KW11 Mechanical Homing
- Homing Repeatability
- KW11 Hardware Limit
- 真实 Mechanical Zero Reference

### 可选扩展

- 轻量 Command Interface
- Modbus RTU
- Parameter Persistence
- IWDG / Task Monitor
- 更完整的 Fault Diagnostics

更多设计边界见 [docs/design.md](docs/design.md)。
