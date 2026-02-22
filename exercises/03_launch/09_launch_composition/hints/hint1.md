# 提示 1

Launch 组合需要两个关键导入：

```python
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
```

`ComposableNodeContainer` 是容器，`ComposableNode` 描述要加载的组件。
容器的 `package` 是 `'rclcpp_components'`，`executable` 是 `'component_container'`。
