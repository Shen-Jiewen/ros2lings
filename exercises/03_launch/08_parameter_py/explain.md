# Python 节点参数 — 声明与使用

## 概念

ROS2 的 Python 节点同样支持参数系统。
参数的声明（`declare_parameter`）和获取（`get_parameter`）必须遵循正确的顺序和类型。

## 声明与获取参数

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')

        # 声明参数（带默认值）
        self.declare_parameter('robot_name', 'default_bot')  # 字符串
        self.declare_parameter('max_count', 10)               # 整数
        self.declare_parameter('timer_period', 1.0)           # 浮点数
        self.declare_parameter('enable_debug', False)          # 布尔

        # 获取参数值
        name = self.get_parameter('robot_name').value
        count = self.get_parameter('max_count').value
        period = self.get_parameter('timer_period').value
        debug = self.get_parameter('enable_debug').value
```

## 类型匹配规则

| 默认值        | 参数类型              | `.value` 类型 |
|--------------|----------------------|--------------|
| `'hello'`    | PARAMETER_STRING     | str          |
| `10`         | PARAMETER_INTEGER    | int          |
| `3.14`       | PARAMETER_DOUBLE     | float        |
| `True`       | PARAMETER_BOOL       | bool         |

## 常见错误

1. **先 get 后 declare** — 必须先 `declare_parameter`，再 `get_parameter`
2. **类型不匹配** — 默认值类型决定参数类型，如 `'ten'` 是字符串不是整数
3. **忘记声明参数** — 未声明的参数不能被获取

## 关键点

- `declare_parameter()` 必须在 `get_parameter()` 之前调用
- 默认值的 Python 类型自动决定参数的 ROS2 类型
- 参数可以通过命令行设置：`ros2 run pkg node --ros-args -p max_count:=20`
