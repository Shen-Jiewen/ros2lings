# Python 订阅者（Subscription）— rclpy 话题接收

## 概念

Subscription（订阅者）负责从指定话题接收消息。当消息到达时，
注册的回调函数会被自动调用。

## 创建订阅者

```python
self.subscription_ = self.create_subscription(
    String,                   # 消息类型
    'chatter',                # 话题名称（必须与发布者一致）
    self.listener_callback,   # 回调函数
    10                        # QoS 队列深度
)
```

## 回调函数

```python
def listener_callback(self, msg):
    self.get_logger().info(f'收到: {msg.data}')
```

注意：作为类的实例方法，第一个参数必须是 `self`，第二个参数才是消息 `msg`。

## 与 C++ 的对比

| C++ (rclcpp)                                          | Python (rclpy)                                        |
|-------------------------------------------------------|-------------------------------------------------------|
| `create_subscription<String>("topic", 10, callback)`  | `create_subscription(String, 'topic', callback, 10)`  |
| `void callback(String::SharedPtr msg)`                | `def callback(self, msg)`                             |
| 回调参数是 `SharedPtr`                                 | 回调参数是消息对象本身                                  |
| QoS 是第二个参数                                       | QoS 是第四个参数                                       |

## 关键点

- 话题名称必须与发布者完全一致，否则无法收到消息
- QoS 深度参数不能省略（C++ 和 Python 的参数顺序不同）
- Python 实例方法的回调必须包含 `self` 参数
- `spin()` 负责触发回调；没有 spin 就不会处理消息
- 建议使用 `self.get_logger().info()` 而非 `print()` 来输出日志
