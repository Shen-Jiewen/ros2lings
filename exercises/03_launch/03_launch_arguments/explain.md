# Launch 参数 — 动态配置启动行为

## 概念

Launch 参数（Launch Arguments）允许在启动时动态传入配置值，
而不需要修改 Launch 文件本身。这让同一个 Launch 文件可以在不同场景下
以不同的配置运行。

例如，可以通过命令行指定话题名称：
```bash
ros2 launch my_package my_launch.py topic_name:=custom_topic
```

## 基本结构

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 声明参数及默认值
    topic_arg = DeclareLaunchArgument(
        'topic_name',
        default_value='hello',
        description='要发布的话题名称',
    )

    # 2. 获取参数值
    topic_config = LaunchConfiguration('topic_name')

    # 3. 将参数传给节点
    node = Node(
        package='my_package',
        executable='my_node.py',
        name='my_node',
        output='screen',
        parameters=[{'topic_name': topic_config}],
    )

    return LaunchDescription([topic_arg, node])
```

## 关键点

- `DeclareLaunchArgument` 声明一个参数，可设置默认值和描述
- `LaunchConfiguration` 引用已声明的参数值（名称必须与声明时一致）
- `parameters` 将配置传递给节点的 ROS 参数（node.get_parameter()）
- 声明必须在引用之前出现在 `LaunchDescription` 中
- 命令行通过 `key:=value` 语法覆盖默认值
