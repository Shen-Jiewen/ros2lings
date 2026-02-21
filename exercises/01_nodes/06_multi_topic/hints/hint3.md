# 提示 3

完整实现：
```cpp
// 构造函数中：
status_pub_ = create_publisher<std_msgs::msg::String>("status", 10);
command_pub_ = create_publisher<std_msgs::msg::String>("command", 10);

status_sub_ = create_subscription<std_msgs::msg::String>(
  "status", 10,
  std::bind(&MultiTopicNode::status_callback, this, std::placeholders::_1));

command_sub_ = create_subscription<std_msgs::msg::String>(
  "command", 10,
  std::bind(&MultiTopicNode::command_callback, this, std::placeholders::_1));

// timer_callback 中取消注释发布代码
```
