# 提示 2

- `rclpy.init()` — 初始化 ROS2 Python 运行时，必须最先调用
- `Node('name')` — 创建节点，必须在 init 之后
- `rclpy.spin_once(node)` — 让节点处理一次回调
