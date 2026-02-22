# 提示 3

完整的实现：

```cpp
// TODO 1: 创建 Buffer 和 TransformListener
tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

// TODO 2: 创建源帧中的点
geometry_msgs::msg::PointStamped point_in;
point_in.header.frame_id = "sensor_link";
point_in.header.stamp = this->now();
point_in.point.x = 1.0;
point_in.point.y = 0.0;
point_in.point.z = 0.0;

// TODO 3: 查询变换
auto transform = tf2_buffer_->lookupTransform(
  "map", "sensor_link", tf2::TimePointZero);

// TODO 4: 执行变换
geometry_msgs::msg::PointStamped point_out;
tf2::doTransform(point_in, point_out, transform);

RCLCPP_INFO(this->get_logger(),
  "sensor_link 中的点 (%.1f, %.1f, %.1f) -> map 中的点 (%.1f, %.1f, %.1f)",
  point_in.point.x, point_in.point.y, point_in.point.z,
  point_out.point.x, point_out.point.y, point_out.point.z);
```
