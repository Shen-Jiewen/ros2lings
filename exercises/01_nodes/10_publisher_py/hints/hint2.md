# 提示 2

- `create_publisher(String, 'chatter', 10)` — 第三个参数是 QoS 队列深度
- `create_timer(0.5, self.timer_callback)` — 回调必须是方法引用，不能是字符串
- `self.publisher_.publish(msg)` — 使用 publish 方法发布消息
