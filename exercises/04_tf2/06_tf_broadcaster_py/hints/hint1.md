# 提示 1

在 Python 中，TF2 提供了两种广播器：
- `tf2_ros.TransformBroadcaster` — 用于**动态**变换（发布到 `/tf`）
- `tf2_ros.StaticTransformBroadcaster` — 用于**静态**变换（发布到 `/tf_static`）

这个练习需要持续发布随时间变化的变换，应该用哪种广播器？

另外，注意检查：
- 时间戳是否被设置了？
- `frame_id` 和 `child_frame_id` 分别代表什么？它们的值正确吗？
