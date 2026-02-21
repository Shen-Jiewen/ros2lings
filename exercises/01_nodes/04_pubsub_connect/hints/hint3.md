# 提示 3

完整的实现：
```cpp
// 构造函数中：
publisher_ = create_publisher<std_msgs::msg::String>("ping", 10);

subscription_ = create_subscription<std_msgs::msg::String>(
  "ping", 10,
  std::bind(&PubSubConnect::sub_callback, this, std::placeholders::_1));

timer_ = create_wall_timer(100ms,
  std::bind(&PubSubConnect::timer_callback, this));

// timer_callback 中：
publisher_->publish(message);

// sub_callback 中：
received_count_++;
```
