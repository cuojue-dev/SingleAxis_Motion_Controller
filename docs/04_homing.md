# 04 Homing

增量式编码器上电后的计数零点，不代表机械机构位于真实零点。因此，系统在执行位置运动前需要通过 HOME/MIN 开关建立机械参考坐标。

第一版 Homing 流程：

```text
IDLE
  |
  | Home Command
  v
HOMING
  |
  | 低速向 HOME/MIN 方向运动
  v
开关触发并完成软件消抖
  |
  v
停止 Motor
  |
  v
将当前位置设为机械参考零点
  |
  v
反向退出开关一小段距离
  |
  v
Homed = true
  |
  v
READY
```