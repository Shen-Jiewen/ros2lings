# 提示 3

完整的修复：
1. 将 `rclpy.init()` 移到 `Node('hello_node_py')` 之前
2. 在 `rclpy.shutdown()` 之前添加 `rclpy.spin_once(node)`
3. 建议在 shutdown 前调用 `node.destroy_node()` 释放资源
