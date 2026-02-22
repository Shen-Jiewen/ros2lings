# 提示 3

完整的实现：

1. 导入条件模块：
   ```python
   from launch.conditions import IfCondition, UnlessCondition
   ```

2. 声明参数：
   ```python
   use_sim_arg = DeclareLaunchArgument('use_sim', default_value='false', description='是否使用仿真模式')
   ```

3. sim_node 添加条件：
   ```python
   condition=IfCondition(LaunchConfiguration('use_sim'))
   ```

4. real_node 添加条件：
   ```python
   condition=UnlessCondition(LaunchConfiguration('use_sim'))
   ```

5. 返回：
   ```python
   return LaunchDescription([use_sim_arg, sim_node, real_node])
   ```
