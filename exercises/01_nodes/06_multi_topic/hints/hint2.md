# 提示 2

- `status_pub_ = create_publisher<std_msgs::msg::String>("status", 10);`
- `command_pub_ = create_publisher<std_msgs::msg::String>("command", 10);`
- 订阅也是类似的模式，每个订阅绑定不同的回调函数
- 注意 `std::bind` 中方法名要对应 `status_callback` 和 `command_callback`
