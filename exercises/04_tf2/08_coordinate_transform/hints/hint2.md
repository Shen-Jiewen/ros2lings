# 提示 2

四个 TODO 的具体内容：

1. **TODO 1**: 创建 Buffer 和 Listener
   ```cpp
   tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
   tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
   ```

2. **TODO 2**: 创建一个 `PointStamped` 点，设置 `frame_id` 为 `"sensor_link"`，
   坐标为 `(1.0, 0.0, 0.0)`。

3. **TODO 3**: `lookupTransform` 的参数顺序是 `(target, source, time)`，
   目标帧是 `"map"`，源帧是 `"sensor_link"`。

4. **TODO 4**: `tf2::doTransform` 接受 3 个参数：输入点、输出点、变换。
