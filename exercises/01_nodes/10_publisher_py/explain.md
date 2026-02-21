# Python 发布者（Publisher）— rclpy 话题发布

## 概念

Publisher（发布者）负责向指定话题发送消息。在 rclpy 中，Publisher 通常与 Timer
配合使用，定时发布消息。

## 创建发布者

```python
self.publisher_ = self.create_publisher(
    String,      # 消息类型
    'chatter',   # 话题名称
    10           # QoS 队列深度
)
```

## 创建定时器

```python
self.timer_ = self.create_timer(
    0.5,                  # 周期（秒）
    self.timer_callback   # 回调函数（必须是可调用对象）
)
```

## 发布消息

```python
msg = String()
msg.data = 'Hello'
self.publisher_.publish(msg)
```

## 与 C++ 的对比

| C++ (rclcpp)                                          | Python (rclpy)                                   |
|-------------------------------------------------------|--------------------------------------------------|
| `create_publisher<String>("topic", 10)`               | `create_publisher(String, 'topic', 10)`          |
| `create_wall_timer(500ms, callback)`                  | `create_timer(0.5, callback)`                    |
| `publisher_->publish(msg)`                            | `publisher_.publish(msg)`                        |
| `std::bind(&Class::method, this)`                     | `self.method`（Python 自动绑定）                  |

## 关键点

- QoS 深度参数不能省略，它决定了消息队列的最大长度
- Python 不需要 `std::bind`，直接传 `self.method` 即可
- `create_timer` 的周期单位是秒（浮点数），C++ 用 `std::chrono`
