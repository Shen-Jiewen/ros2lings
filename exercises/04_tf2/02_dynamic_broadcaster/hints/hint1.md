# 提示 1

注意区分两种广播器：
- `tf2_ros::TransformBroadcaster` — 用于**动态**变换（发布到 `/tf`）
- `tf2_ros::StaticTransformBroadcaster` — 用于**静态**变换（发布到 `/tf_static`）

这个练习中我们需要持续发布随时间变化的变换，应该用哪种广播器？

另外，检查定时器回调中的时间戳和 `sendTransform` 的调用情况。
