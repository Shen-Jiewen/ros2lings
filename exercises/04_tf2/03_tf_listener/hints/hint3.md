# 提示 3

完整的修复：

1. 给 Buffer 构造函数传入时钟：
   ```cpp
   tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
   ```

2. 修正 `lookupTransform` 的帧参数顺序：
   ```cpp
   t = tf_buffer_->lookupTransform(
     "base_link",       // 目标帧（target_frame）
     "sensor_link",     // 源帧（source_frame）
     tf2::TimePointZero);
   ```

3. 将 `lookupTransform` 调用包裹在 try-catch 中：
   ```cpp
   try {
     geometry_msgs::msg::TransformStamped t;
     t = tf_buffer_->lookupTransform(
       "base_link", "sensor_link", tf2::TimePointZero);
     RCLCPP_INFO(this->get_logger(),
       "变换 base_link -> sensor_link: [%.2f, %.2f, %.2f]",
       t.transform.translation.x,
       t.transform.translation.y,
       t.transform.translation.z);
   } catch (const tf2::TransformException & ex) {
     RCLCPP_WARN(this->get_logger(), "无法获取变换: %s", ex.what());
   }
   ```
