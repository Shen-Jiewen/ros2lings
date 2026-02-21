# 提示 2

- `create_publisher<std_msgs::msg::String>("话题名", QoS深度)` — 创建发布者需要模板参数和两个函数参数
- `create_wall_timer(500ms, callback)` — 定时器周期需要 `std::chrono` 类型，不能用纯整数
- 发布消息使用 `publisher_->publish(message)`
