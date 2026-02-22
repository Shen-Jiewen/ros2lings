# 提示 2

三个错误分别是：

1. 导入了 `StaticTransformBroadcaster`，应该导入 `TransformBroadcaster`。
   动态变换需要动态广播器，静态广播器只用于一次性的固定变换。

2. `t.header.stamp` 没有被设置，时间戳默认为零。
   动态变换每次发布时必须更新时间戳为当前时间。

3. `t.header.frame_id` 和 `t.child_frame_id` 的值设反了：
   - `frame_id` 应该是父帧 `'world'`
   - `child_frame_id` 应该是子帧 `'child_frame'`
