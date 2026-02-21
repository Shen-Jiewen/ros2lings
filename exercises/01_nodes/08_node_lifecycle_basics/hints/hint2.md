# 提示 2

- `get_name()` 返回创建时传入的名称字符串
- 默认命名空间是根命名空间 "/"
- `rclcpp::init()` 初始化 ROS2 的 Context（上下文）
- `rclcpp::spin()` 启动事件循环来处理回调
- 每个节点默认会创建 `/rosout` 话题用于日志
