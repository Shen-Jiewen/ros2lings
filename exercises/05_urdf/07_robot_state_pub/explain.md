# Robot State Publisher — 将 URDF 加载到 ROS2 中

## 概念

在 ROS2 中，`robot_state_publisher` 是将 URDF 机器人模型与 TF2 坐标变换系统
连接起来的核心节点。它读取 URDF 描述，并根据关节状态实时发布各 link 之间的
坐标变换关系。

## robot_state_publisher 的作用

`robot_state_publisher` 节点的核心功能：

1. **读取 URDF 模型** — 通过 `robot_description` 参数接收 URDF 字符串
2. **发布静态变换** — 将 fixed joint 的变换发布到 `/tf_static`
3. **发布动态变换** — 订阅 `/joint_states`，将可动关节的变换发布到 `/tf`
4. **提供模型描述** — 将 URDF 发布到 `/robot_description` 话题

## 在 Launch 文件中启动

### 基本步骤

```python
import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 构建 URDF 文件路径
    urdf_path = os.path.join(
        os.path.dirname(__file__), '..', 'urdf', 'robot.urdf'
    )

    # 2. 读取 URDF 文件内容为字符串
    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    # 3. 创建 robot_state_publisher 节点
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    # 4. 返回 LaunchDescription
    return LaunchDescription([robot_state_pub_node])
```

### 关键点

- **URDF 以字符串形式传递** — 不是文件路径，而是整个 URDF 文件的文本内容
- **参数名必须是 `robot_description`** — 这是 `robot_state_publisher` 期望的参数名
- **路径构建** — 使用 `os.path.dirname(__file__)` 获取 launch 文件所在目录，
  然后用相对路径定位 URDF 文件

## 文件路径技巧

在 Launch 文件中，`__file__` 指向当前 launch 文件的路径。通过 `os.path.join`
和 `..` 可以向上导航到包的根目录：

```python
# launch/display.launch.py 中
# __file__ = .../launch/display.launch.py
# os.path.dirname(__file__) = .../launch/
# '..', 'urdf', 'robot.urdf' = .../urdf/robot.urdf
urdf_path = os.path.join(os.path.dirname(__file__), '..', 'urdf', 'robot.urdf')
```

## 命令行等效操作

```bash
# 直接用命令行启动 robot_state_publisher
ros2 run robot_state_publisher robot_state_publisher \
    --ros-args -p robot_description:="$(cat urdf/robot.urdf)"
```

## 常见错误

1. **传递文件路径而非内容** — `robot_description` 参数需要的是 URDF 字符串，不是文件路径
2. **路径拼接错误** — 忘记使用 `os.path.dirname(__file__)` 导致相对路径不正确
3. **LaunchDescription 为空** — 忘记将创建的 Node 加入返回列表
4. **忘记读取文件** — 只构建了路径但没有用 `open()` 读取内容
