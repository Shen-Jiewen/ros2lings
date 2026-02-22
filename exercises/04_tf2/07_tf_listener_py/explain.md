# Python TF2 变换监听 — Buffer 与 TransformListener

## 概念

在 Python 中使用 TF2 监听和查询坐标变换，需要配合两个核心组件：
- **Buffer** — 存储所有已知的坐标变换关系
- **TransformListener** — 自动订阅 `/tf` 和 `/tf_static`，将变换数据写入 Buffer

当你需要查询两个坐标帧之间的关系时，通过 Buffer 的 `lookup_transform` 方法获取。

## Python 基本用法

```python
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
import tf2_ros


class TfListenerNode(Node):

    def __init__(self):
        super().__init__('tf_listener')

        # 创建 Buffer — 传入 node 参数
        self.buffer_ = Buffer(node=self)

        # 创建 TransformListener — 传入 buffer 和 node
        self.listener_ = TransformListener(self.buffer_, self)

        # 定时查询变换
        self.timer_ = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        try:
            # lookup_transform(target_frame, source_frame, time)
            t = self.buffer_.lookup_transform(
                'world',           # 目标帧
                'child_frame',     # 源帧
                rclpy.time.Time())  # 最新可用时间

            self.get_logger().info(
                f'x={t.transform.translation.x:.2f}, '
                f'y={t.transform.translation.y:.2f}')

        except tf2_ros.TransformException as ex:
            self.get_logger().warn(f'变换查询失败: {ex}')
```

## 关键点

### 1. Buffer 和 TransformListener 的创建顺序

必须先创建 Buffer，再将它传给 TransformListener：
```python
# 正确
self.buffer_ = Buffer(node=self)
self.listener_ = TransformListener(self.buffer_, self)

# 错误 — listener 没有收到 buffer
self.listener_ = TransformListener(node=self)
```

TransformListener 需要知道将收到的变换数据存到哪个 Buffer 中。
如果不传入 buffer，listener 会创建自己的内部 buffer，
导致你后续查询的 buffer 中没有数据。

### 2. lookup_transform 参数顺序

```python
# lookup_transform(target_frame, source_frame, time)
# 含义：查询 source_frame 在 target_frame 坐标系中的位姿
t = self.buffer_.lookup_transform('world', 'child_frame', rclpy.time.Time())
# 结果表示 child_frame 在 world 坐标系中的位姿
```

**注意参数顺序！** 第一个参数是目标帧（target），第二个是源帧（source）。
这和 "从 A 到 B" 的直觉顺序可能相反。

### 3. 异常处理不可省略

`lookup_transform` 可能在以下情况抛出异常：
- `tf2_ros.LookupException` — 请求的帧不存在
- `tf2_ros.ConnectivityException` — 两个帧之间没有连接路径
- `tf2_ros.ExtrapolationException` — 请求的时间超出缓冲区范围

这些异常都继承自 `tf2_ros.TransformException`，所以通常用：
```python
try:
    t = self.buffer_.lookup_transform(...)
except tf2_ros.TransformException as ex:
    self.get_logger().warn(f'{ex}')
```

**不要**捕获 `ValueError` 或其他 Python 内置异常类型 — TF2 抛出的是专门的异常类。

### 4. Python 与 C++ 的区别

| 操作 | C++ | Python |
|------|-----|--------|
| 创建 Buffer | `Buffer(this->get_clock())` | `Buffer(node=self)` |
| 创建 Listener | `TransformListener(*buffer)` | `TransformListener(buffer, self)` |
| 查询变换 | `buffer->lookupTransform(...)` | `buffer_.lookup_transform(...)` |
| 时间参数 | `tf2::TimePointZero` | `rclpy.time.Time()` |
| 异常基类 | `tf2::TransformException` | `tf2_ros.TransformException` |
