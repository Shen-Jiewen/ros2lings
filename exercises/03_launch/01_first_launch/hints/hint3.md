# 提示 3

完整的修复：
1. 把 `package='wrong_package_name'` 改为 `package='ros2lings_28_first_launch'`
2. 把 `executable='wrong_node'` 改为 `executable='first_launch_node.py'`
3. 把 `return LaunchDescription([])` 改为 `return LaunchDescription([node])`
