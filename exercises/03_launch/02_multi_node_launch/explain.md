# 多节点 Launch — 同时启动多个节点

## 概念

一个 Launch 文件可以同时启动多个节点。只需在 `LaunchDescription` 的列表中
添加多个 `Node` action 即可。这是 Launch 文件最基本也是最常用的功能。

在实际的机器人系统中，通常需要同时运行传感器驱动、控制器、规划器等
多个节点，用一个 Launch 文件可以一键启动整个系统。

## 基本结构

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    talker = Node(
        package='my_package',
        executable='talker_node.py',
        name='talker',               # 每个节点需要唯一的名称
        output='screen',
    )

    listener = Node(
        package='my_package',
        executable='listener_node.py',
        name='listener',             # 名称不能与其他节点重复
        output='screen',
    )

    return LaunchDescription([talker, listener])
```

## 关键点

- `LaunchDescription` 的列表中可以包含任意数量的 `Node` action
- 每个节点的 `name` 必须唯一，否则后启动的节点会替换先启动的
- `output='screen'` 让节点日志显示在终端，方便调试
- 发布者和订阅者使用相同的话题名称即可实现通信
