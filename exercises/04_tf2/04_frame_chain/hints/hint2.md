# 提示 2

四个 TODO 的具体内容：

1. **TODO 1**: 创建广播器
   ```cpp
   tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
   ```

2. **TODO 2**: 创建 `map -> odom` 变换，注意设置 `rotation.w = 1.0`。

3. **TODO 3**: 参照 TODO 2 的模式，创建另外两个 `TransformStamped`，
   分别是 `odom -> base_link` 和 `base_link -> sensor_link`，
   各自设置正确的 `frame_id`、`child_frame_id` 和 `translation`。

4. **TODO 4**: 用 `sendTransform` 把 `transforms` 向量发送出去。
