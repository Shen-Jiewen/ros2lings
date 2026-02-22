# 提示 1

条件启动需要三个步骤：

1. 用 `DeclareLaunchArgument` 声明参数
2. 用 `LaunchConfiguration` 获取参数值
3. 在 `Node` 中用 `condition=IfCondition(...)` 或 `condition=UnlessCondition(...)` 设置条件

别忘了导入 `IfCondition` 和 `UnlessCondition`：
```python
from launch.conditions import IfCondition, UnlessCondition
```
