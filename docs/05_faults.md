# 05 Faults

Project25 第一版优先定义能够实际制造和验证的故障。

| Fault | Detection | System Action | Recovery |
| --- | --- | --- | --- |
| FAULT_HARD_LIMIT | 普通运动过程中触发 MIN 或 MAX 硬件限位 | 停止 Motor，进入 FAULT，丢弃当前运动命令 | 确认机构离开危险位置并执行 Fault Reset，必要时重新 Homing |
| FAULT_SOFT_LIMIT | 目标位置超出允许的软件位置范围 | 拒绝运动命令，Motor 不启动，并报告 Fault | 修改为合法目标位置后执行 Fault Reset |
| FAULT_COMM_TIMEOUT | 运动过程中超过规定时间未收到有效控制命令 | 停止 Motor，进入 FAULT，禁止自动继续旧命令 | 通信恢复后执行 Fault Reset，再发送新命令 |
| FAULT_MOTION_FEEDBACK | 已输出运动控制量，但规定时间内 Encoder 反馈没有合理变化 | 停止 Motor 并进入 FAULT | 检查机械、电机和 Encoder，排除问题后执行 Fault Reset，并重新 Homing |

通用处理原则：

- 进入 FAULT 后必须关闭 Motor 输出
- 故障条件仍然存在时不得完成 Fault Reset
- Fault Reset 后返回 IDLE
- 不得自动继续故障前的运动命令
- 可能导致机械坐标不可信的故障应清除 Homed 状态

第一版不分别判断 `STALL` 和 `ENCODER_BROKEN`。因为“有控制输出但 Encoder 不变化”既可能是机械堵转，也可能是编码器断线，所以先统一报告 `FAULT_MOTION_FEEDBACK`。
