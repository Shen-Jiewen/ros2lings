# Launch 组合 — 在 Launch 文件中加载组件节点

## 概念

通过 Launch 文件，可以将多个组件节点加载到同一个进程（容器）中运行。
这比手动使用 `ros2 component load` 更方便，适合系统启动时批量加载组件。

## 关键组件

### ComposableNodeContainer

组件容器，相当于运行 `rclcpp_components::component_container`：

```python
from launch_ros.actions import ComposableNodeContainer

container = ComposableNodeContainer(
    name='my_container',
    namespace='',
    package='rclcpp_components',
    executable='component_container',
    composable_node_descriptions=[...],  # 组件列表
    output='screen',
)
```

### ComposableNode

描述要加载到容器中的组件节点：

```python
from launch_ros.descriptions import ComposableNode

node = ComposableNode(
    package='my_package',          # 组件所在的包
    plugin='MyComponentClass',     # 注册时的类名
    name='my_node_name',           # 运行时的节点名
)
```

## C++ 组件要求

组件必须满足：
1. 继承 `rclcpp::Node`
2. 构造函数接受 `const rclcpp::NodeOptions &`
3. 编译为共享库（`add_library(SHARED ...)`）
4. 注册组件（`RCLCPP_COMPONENTS_REGISTER_NODE(ClassName)`）
5. CMakeLists 中调用 `rclcpp_components_register_nodes()`

## 完整示例

```python
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container = ComposableNodeContainer(
        name='container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='my_pkg',
                plugin='PublisherComponent',
                name='publisher',
            ),
            ComposableNode(
                package='my_pkg',
                plugin='SubscriberComponent',
                name='subscriber',
            ),
        ],
        output='screen',
    )
    return LaunchDescription([container])
```

## 优势

- 组件在同一进程中共享内存，通信效率更高
- Launch 文件管理组件比手动 `ros2 component load` 更方便
- 支持在启动时传递参数和重映射
