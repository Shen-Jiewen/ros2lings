# 提示 2

三个错误分别是：

1. 成员变量 `tf_broadcaster_` 的类型是 `StaticTransformBroadcaster`，
   应该改为 `TransformBroadcaster`。注意构造函数和成员声明都要改。

2. 在 `broadcast_timer_callback()` 中，`t.header.stamp` 没有被设置，
   应该使用 `this->now()` 获取当前时间戳。

3. `tf_broadcaster_->sendTransform(t)` 被注释掉了，
   变换永远不会被发布出去。
