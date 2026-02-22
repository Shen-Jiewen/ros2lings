# 提示 3

完整的实现：

```python
def generate_launch_description():
    urdf_path = os.path.join(
        os.path.dirname(__file__), '..', 'urdf', 'robot.urdf'
    )

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    return LaunchDescription([robot_state_pub_node])
```
