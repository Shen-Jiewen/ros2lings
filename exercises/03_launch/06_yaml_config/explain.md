# YAML 参数配置 — 从文件加载节点参数

## 概念

ROS2 支持从 YAML 文件加载节点参数，避免在 Launch 文件中硬编码参数值。
这种方式特别适合管理大量参数，或在不同环境下使用不同配置。

## YAML 文件格式

```yaml
节点名称:
  ros__parameters:
    参数名1: 值1
    参数名2: 值2
```

注意：`ros__parameters` 中间是**两个下划线**。

示例：

```yaml
configurable_node:
  ros__parameters:
    robot_name: "ros2bot"
    max_speed: 1.5
    enable_logging: true
```

## 在 Launch 文件中加载 YAML

```python
import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 构建 YAML 文件路径
    yaml_path = os.path.join(
        os.path.dirname(__file__), '..', 'config', 'params.yaml'
    )

    node = Node(
        package='my_package',
        executable='my_node.py',
        name='configurable_node',      # 必须与 YAML 中的节点名一致
        output='screen',
        parameters=[yaml_path],         # 加载 YAML 文件
    )

    return LaunchDescription([node])
```

## 关键点

- YAML 中的节点名必须与 `Node` action 的 `name` 参数一致
- `parameters` 接受一个列表，可以同时加载多个 YAML 文件或字典
- `os.path.join` + `os.path.dirname(__file__)` 是构建相对路径的标准方式
- 节点必须先 `declare_parameter` 才能使用 YAML 中的参数值
- YAML 中的参数类型会自动匹配（整数、浮点数、字符串、布尔值）
