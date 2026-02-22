# Launch 事件系统 — 事件驱动的启动控制

## 概念

ROS2 Launch 框架内置了一个事件系统，可以在特定事件发生时执行相应的操作。
这使得 Launch 文件不仅能同时启动多个节点，还能实现复杂的启动逻辑，
比如"节点 A 退出后再启动节点 B"。

## 常见事件

| 事件类 | 触发时机 |
|-------|---------|
| `OnProcessStart` | 进程启动时 |
| `OnProcessExit` | 进程退出时 |
| `OnProcessIO` | 进程有输入/输出时 |
| `OnShutdown` | Launch 系统关闭时 |
| `OnExecutionComplete` | Action 执行完成时 |

## 注册事件处理器

使用 `RegisterEventHandler` 将事件处理器加入 Launch 描述中：

```python
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.actions import EmitEvent, Shutdown
from launch_ros.actions import Node

def generate_launch_description():
    node_a = Node(package='pkg', executable='node_a', name='node_a')
    node_b = Node(package='pkg', executable='node_b', name='node_b')

    # 当 node_a 退出后，启动 node_b
    event_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=node_a,
            on_exit=[node_b],
        )
    )

    return LaunchDescription([node_a, event_handler])
```

## 链式启动

通过 `OnProcessExit` 可以实现"A 退出后启动 B"：

```python
RegisterEventHandler(
    OnProcessExit(
        target_action=node_a,
        on_exit=[node_b],
    )
)
```

## Shutdown Action

`Shutdown` action 可以在事件触发时关闭整个 Launch 系统：

```python
from launch.actions import Shutdown

RegisterEventHandler(
    OnProcessExit(
        target_action=critical_node,
        on_exit=[Shutdown(reason='Critical node exited')],
    )
)
```

## 退出码访问

`OnProcessExit` 事件处理器可以通过回调函数访问进程的退出码：

```python
def on_exit_callback(event, context):
    print(f'Process exited with code: {event.returncode}')
    return []
```

## 关键点

- `RegisterEventHandler` 用于将事件处理器加入 LaunchDescription
- `OnProcessExit` 是最常用的事件，可以实现链式启动
- `Shutdown` action 用于在事件触发时关闭整个 launch
- 事件处理器可以访问事件的详细信息（如退出码）
