# Launch 文件 — 批量启动节点的工具

## 概念

Launch 文件是 ROS2 中用来同时启动和配置多个节点的脚本。
与手动逐个 `ros2 run` 不同，Launch 文件可以一次性启动整个系统，
并配置节点的参数、命名空间、话题重映射等。

ROS2 支持 Python、XML、YAML 三种格式的 Launch 文件，
其中 Python 格式最灵活，也是最常用的。

## 基本结构

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_package',        # 包名
            executable='my_node',        # 可执行文件名
            name='my_node_name',         # 节点名称
            output='screen',             # 输出到终端
        ),
    ])
```

## Node Action

`Node` 是最常用的 Launch Action，用来启动一个 ROS2 节点：

| 参数          | 说明                             |
|---------------|----------------------------------|
| `package`     | 节点所在的 ROS2 包名             |
| `executable`  | 可执行文件名（CMakeLists 中安装的名称）|
| `name`        | 运行时节点名称（覆盖代码中的默认名）|
| `output`      | 输出模式：`'screen'` 或 `'log'`  |
| `parameters`  | 参数列表（字典或 YAML 文件路径） |
| `remappings`  | 话题/服务重映射列表              |

## 关键点

- `generate_launch_description()` 是 Launch 文件的入口函数，必须返回 `LaunchDescription`
- `LaunchDescription` 中的列表不能为空，否则不会启动任何节点
- `package` 和 `executable` 必须与实际安装的名称完全匹配
- `output='screen'` 让节点日志输出到终端，方便调试
