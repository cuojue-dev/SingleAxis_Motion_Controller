# 03 State Machine

Project25 第一版使用以下状态：

```text
BOOT
  |
  v
INIT
  |
  v
IDLE
  |
  | Home Command
  v
HOMING
  |
  v
READY
  |
  +------------+
  |            |
  v            v
RUN_SPEED   RUN_POSITION
  |            |
  +-----+------+
        |
        v
      READY
```

HOMING、RUN_SPEED 或 RUN_POSITION 检测到故障后进入 `FAULT`，Motor 必须停止。

Fault Reset 成功后返回 `IDLE`，不得自动继续故障前的运动命令。

位置运动只能在 Homing 成功、机械坐标有效后执行。