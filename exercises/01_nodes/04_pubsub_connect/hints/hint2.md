# 提示 2

- Publisher: `create_publisher<std_msgs::msg::String>("ping", 10)`
- Subscription: `create_subscription<std_msgs::msg::String>("ping", 10, callback)`
- Timer: `create_wall_timer(100ms, callback)`
- 回调绑定: `std::bind(&PubSubConnect::方法名, this, ...)`
