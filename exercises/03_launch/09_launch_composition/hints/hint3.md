# 提示 3

完整实现：

```python
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container = ComposableNodeContainer(
        name='composition_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='ros2lings_36_launch_composition',
                plugin='PublisherComponent',
                name='publisher_component',
            ),
            ComposableNode(
                package='ros2lings_36_launch_composition',
                plugin='SubscriberComponent',
                name='subscriber_component',
            ),
        ],
        output='screen',
    )
    return LaunchDescription([container])
```
