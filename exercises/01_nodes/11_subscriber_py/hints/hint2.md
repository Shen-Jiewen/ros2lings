# 提示 2

- 回调函数签名: `def listener_callback(self, msg)` — 别忘了 `self`
- `create_subscription` 需要四个参数: `(消息类型, '话题名', 回调, QoS深度)`
- 话题名称应该与发布者使用的一致，通常是 `'chatter'`
