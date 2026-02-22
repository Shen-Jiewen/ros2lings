# Python TF2 动态变换广播 — TransformBroadcaster

## 概念

在 Python 中使用 TF2 发布动态坐标变换，与 C++ 的逻辑完全一致，
只是 API 风格遵循 Python 的惯例。

**动态变换**描述的是随时间变化的空间关系，例如移动的机器人、旋转的传感器等。
必须在定时器回调中持续发布，并且每次都要更新时间戳。

## TransformBroadcaster vs StaticTransformBroadcaster

| 特性 | TransformBroadcaster | StaticTransformBroadcaster |
|------|---------------------|---------------------------|
| 话题 | `/tf` | `/tf_static` |
| 发布方式 | 持续发布（定时器回调） | 只需发布一次 |
| 适用场景 | 运动部件、移动平台 | 固定安装的传感器 |
| Python 模块 | `tf2_ros.TransformBroadcaster` | `tf2_ros.StaticTransformBroadcaster` |

## Python 基本用法

```python
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class MyBroadcaster(Node):

    def __init__(self):
        super().__init__('my_broadcaster')

        # 创建动态变换广播器
        self.broadcaster_ = TransformBroadcaster(self)

        # 定时器，每 100ms 发布一次
        self.timer_ = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        t = TransformStamped()

        # 关键：每次回调都要更新时间戳！
        t.header.stamp = self.get_clock().now().to_msg()

        # 父帧和子帧
        t.header.frame_id = 'world'        # 父帧
        t.child_frame_id = 'child_frame'   # 子帧

        # 设置随时间变化的平移
        seconds = self.get_clock().now().nanoseconds / 1e9
        t.transform.translation.x = math.cos(seconds)
        t.transform.translation.y = math.sin(seconds)
        t.transform.translation.z = 0.0

        # 单位四元数（无旋转）
        t.transform.rotation.w = 1.0

        # 发布变换
        self.broadcaster_.sendTransform(t)
```

## 关键点

### 1. 选择正确的广播器类型
- 动态变换（随时间变化）使用 `TransformBroadcaster`
- 静态变换（固定不变）使用 `StaticTransformBroadcaster`
- 用错类型不会直接报错，但行为会异常

### 2. 时间戳必须设置
每次发布动态变换时，必须将 `header.stamp` 设为当前时间：
```python
t.header.stamp = self.get_clock().now().to_msg()
```
如果不设置时间戳（默认为零），TF2 的查询会因时间不匹配而失败。

### 3. frame_id 与 child_frame_id 的方向
- `header.frame_id` = 父帧（参考帧）
- `child_frame_id` = 子帧（被描述的帧）
- 变换含义：子帧**相对于**父帧的位姿
- 写反会导致 TF 树结构错误

### 4. Python 与 C++ 的区别
| 操作 | C++ | Python |
|------|-----|--------|
| 获取当前时间 | `this->now()` | `self.get_clock().now().to_msg()` |
| 创建广播器 | `TransformBroadcaster(this)` | `TransformBroadcaster(self)` |
| 发布变换 | `sendTransform(t)` | `sendTransform(t)` |
