# 25_SingleAxis_Motion_Controller

这是一个基于 STM32F103、FreeRTOS、N20 有刷直流减速电机和 AB 增量编码器的**单轴闭环运动控制学习项目**。

项目计划实现一个缩小版但结构完整的运动控制节点：系统上电后通过 Homing 建立机械参考坐标，接收速度或位置命令，在软硬件限位和故障保护下驱动电机完成闭环运动。

机械摇臂只作为 Homing、限位、位置控制和重复定位的测试装置，不是项目本体。

## 为什么做这个项目

Project24 已经完成了 N20 电机的速度闭环和位置—速度串级控制，验证了基本控制算法能够驱动电机到达目标速度和目标位置。

Project25 不再以继续研究 PID 公式为主要目标，而是学习如何把已有的控制能力组织成一个完整的嵌入式运动控制系统，包括状态管理、运动命令、Homing、限位、故障处理、通信、参数管理和系统监控。

项目希望训练的不只是“让电机转起来”，而是理解一个运动控制设备如何从上电初始化、建立坐标、接收命令和执行运动，一直运行到异常检测与安全停止。

## 硬件与软件平台

### 硬件

- STM32F103ZET6
- 正点原子精英 STM32F103 开发板 V2.6
- N20 6 V 有刷直流减速电机
- AB 增量编码器
- DRV8833 电机驱动模块
- KW11 微动开关，计划用作 HOME/MIN 和 MAX 硬件限位
- 自制单轴机械测试台架

第一版计划继续采用 Project24 已经验证过的 5 V 电机供电条件。

### 软件

- STM32CubeMX
- STM32CubeIDE
- STM32 HAL
- FreeRTOS + CMSIS-RTOS2
- Git
- VS Code

STM32CubeIDE 负责工程配置、编译、下载和调试；VS Code 主要用于文档编写、工程浏览、搜索和 Git 操作。

## v1.0 计划能力

- Speed Mode（速度模式）
- Position Mode（位置模式）
- Homing（回零并建立机械坐标）
- MIN/MAX 硬件限位
- Software Position Limit（软件位置限位）
- 运动状态机
- Trapezoidal Motion Profile（梯形运动轨迹）
- 故障检测与安全停止
- RS485 / Modbus RTU 控制与状态读取
- 运行参数管理
- FreeRTOS Task 健康监控与 IWDG

通信模块只负责产生运动命令，电机输出仍由运动控制模块统一管理。

## 当前开发状态

项目目前处于 Phase 0 设计阶段。

已经完成：

- 建立独立 Git 仓库
- 创建 README 和初始设计文档目录
- 确定项目定位、第一版范围和阶段开发路线
- 确定以 Project24 作为控制算法原型

尚未开始：

- 创建 STM32CubeIDE / FreeRTOS 固件工程
- 迁移 Motor、Encoder 和 Controller 模块
- 编译、下载和 Project25 上板验证

因此，本 README 中列出的 v1.0 能力均为计划目标，不代表当前已经实现。