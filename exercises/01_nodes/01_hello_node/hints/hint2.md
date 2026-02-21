# 提示 2

- `rclcpp::init(argc, argv)` — 传入命令行参数
- 创建节点要用 `std::make_shared<rclcpp::Node>("name")`
- 节点需要 `rclcpp::spin_some(node)` 才能运行
