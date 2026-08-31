# 02 Architecture

## 1. 文档目的

本文档描述 Project25 的软件组成、主要数据流、模块职责和 Task 演进方式。

架构设计的目标是保证运动控制、通信、故障处理和系统监控之间职责清晰，避免多个模块同时操作 Motor 或修改运动状态。

本文档只定义模块边界和协作关系，不规定所有函数、数据结构和 FreeRTOS API 的具体实现。

## 2. 架构原则

Project25 采用以下基本原则：

- Motor 输出只能由运动控制核心统一管理
- 通信模块只能产生 Command，不能直接修改 PWM
- Driver 只负责硬件访问，不负责决定系统状态
- Controller 根据输入计算输出，不直接读取硬件
- Fault、Parameter 和 Monitor 是普通功能模块，不因为功能独立就必须创建单独 Task
- Task 是否独立取决于调度需求，而不是文件模块数量
- 功能按照实际开发阶段逐步加入，不一次创建所有空模块

## 3. 系统数据流

系统的主要控制链路为：

```text
External Controller
        |
        v
RS485 / Modbus RTU
        |
        v
Command
        |
        v
Motion Manager / State Machine
        |
        v
Trajectory
        |
        v
Position Controller
        |
        v
Speed Controller
        |
        v
Motor Driver
        |
        v
DRV8833 + N20 Motor
        |
        v
Mechanical Axis
        |
        v
AB Encoder Feedback
        |
        +--------------------------+
                                   |
                                   v
                    Position / Speed Feedback
```

## 4. 模块职责

| 模块 | 主要职责 | 不负责 |
| --- | --- | --- |
| Motion Manager | 管理运动状态并统一调度控制链 | 解析 Modbus、保存参数 |
| Fault Manager | 记录、查询和清除 Fault | 直接修改 PWM |
| Communication | 解析 Modbus、生成 Command、返回 Status | 直接控制 Motor |
| Trajectory | 生成受速度和加速度限制的运动目标 | 读取硬件、修改 PWM |
| Position Controller | 根据位置误差生成速度目标 | 直接控制 Motor |
| Speed Controller | 根据速度误差生成控制输出 | 决定系统状态 |
| Motor Driver | 设置 DRV8833 方向和 PWM | 判断运动模式 |
| Encoder Driver | 提供位置和速度反馈 | 执行闭环控制 |
| Limit Switch Driver | 读取并消抖限位开关 | 决定完整 Homing 流程 |
| Parameter Manager | 管理参数检查和持久化 | 执行运动控制 |
| System Monitor | 监控关键 Task 并管理 IWDG 刷新条件 | 执行运动命令 |

架构中列出的模块按照开发阶段逐步建立，不代表当前已经实现。

## 5. FreeRTOS Task 演进

第一阶段只创建 `motion_task`，负责周期执行运动控制。

通信和系统监控加入后，再根据独立调度需求增加 `communication_task` 和 `system_monitor_task`。Motor 始终由 `motion_task` 统一管理。