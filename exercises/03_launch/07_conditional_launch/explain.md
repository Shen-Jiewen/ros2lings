# 条件启动 — IfCondition 与 UnlessCondition

## 概念

ROS2 Launch 文件不仅能同时启动多个节点，还能根据条件决定是否启动某个节点。
这在需要区分仿真模式和真实硬件模式、或根据配置动态选择节点时非常有用。

## 关键组件

### DeclareLaunchArgument

声明一个 Launch 参数，可以在命令行中传入：

```python
from launch.actions import DeclareLaunchArgument

use_sim_arg = DeclareLaunchArgument(
    'use_sim',
    default_value='false',
    description='是否使用仿真模式'
)
```

### LaunchConfiguration

获取 Launch 参数的值：

```python
from launch.substitutions import LaunchConfiguration

use_sim = LaunchConfiguration('use_sim')
```

### IfCondition / UnlessCondition

根据条件决定是否执行某个 Action：

```python
from launch.conditions import IfCondition, UnlessCondition

# 当 use_sim='true' 时启动
sim_node = Node(..., condition=IfCondition(use_sim))

# 当 use_sim='false' 时启动（即 use_sim 不为 true）
real_node = Node(..., condition=UnlessCondition(use_sim))
```

## 使用方式

```bash
# 默认使用真实硬件模式
ros2 launch my_pkg my_launch.py

# 使用仿真模式
ros2 launch my_pkg my_launch.py use_sim:=true
```

## 关键点

- `IfCondition` 在条件为 `'true'` 时执行
- `UnlessCondition` 在条件为 `'false'` 时执行（即条件不满足时执行）
- 条件值是字符串 `'true'` 或 `'false'`，不是 Python 布尔值
- `DeclareLaunchArgument` 必须加入 `LaunchDescription` 列表中
- 条件节点也必须加入 `LaunchDescription` 列表中
