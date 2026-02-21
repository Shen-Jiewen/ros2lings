# 提示 3

完整的修复：
1. `rclcpp::init(argc, argv);`
2. `auto node = std::make_shared<rclcpp::Node>("hello_node");`
3. 在 shutdown 前加上 `rclcpp::spin_some(node);`
