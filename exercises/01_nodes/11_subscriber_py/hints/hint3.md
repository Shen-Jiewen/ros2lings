# 提示 3

完整的修复：
1. `def listener_callback(self, msg):` — 添加 `self` 参数
2. 话题名从 `'wrong_topic'` 改为 `'chatter'`
3. 添加 QoS 深度参数: `self.create_subscription(String, 'chatter', self.listener_callback, 10)`
4. 建议使用 `self.get_logger().info()` 代替 `print()`
