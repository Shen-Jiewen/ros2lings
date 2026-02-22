# 提示 3

完整的实现：

```python
def generate_launch_description():
    yaml_path = os.path.join(
        os.path.dirname(__file__), '..', 'config', 'params.yaml'
    )

    node = Node(
        package='ros2lings_33_yaml_config',
        executable='configurable_node.py',
        name='configurable_node',
        output='screen',
        parameters=[yaml_path],
    )

    return LaunchDescription([node])
```
