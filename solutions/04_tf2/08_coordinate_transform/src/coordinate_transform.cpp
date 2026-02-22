#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

class CoordinateTransformNode : public rclcpp::Node
{
public:
  CoordinateTransformNode() : Node("coordinate_transform_node")
  {
    // 创建 tf2 Buffer 和 TransformListener
    tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

    // 创建定时器，每秒尝试一次坐标变换
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&CoordinateTransformNode::timer_callback, this));
  }

private:
  void timer_callback()
  {
    // 创建一个在 sensor_link 帧中的点
    geometry_msgs::msg::PointStamped point_in;
    point_in.header.frame_id = "sensor_link";
    point_in.header.stamp = this->now();
    point_in.point.x = 1.0;
    point_in.point.y = 0.0;
    point_in.point.z = 0.0;

    try {
      // 查询从 sensor_link 到 map 的变换
      auto transform = tf2_buffer_->lookupTransform(
        "map", "sensor_link", tf2::TimePointZero);

      // 执行坐标变换
      geometry_msgs::msg::PointStamped point_out;
      tf2::doTransform(point_in, point_out, transform);

      RCLCPP_INFO(this->get_logger(),
        "sensor_link 中的点 (%.1f, %.1f, %.1f) -> map 中的点 (%.1f, %.1f, %.1f)",
        point_in.point.x, point_in.point.y, point_in.point.z,
        point_out.point.x, point_out.point.y, point_out.point.z);

    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "坐标变换失败: %s", ex.what());
    }
  }

  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
  rclcpp::TimerBase::SharedPtr timer_;

#ifdef ROS2LINGS_TEST
public:
  std::shared_ptr<tf2_ros::Buffer> get_buffer() const { return tf2_buffer_; }
#endif
};

#ifndef ROS2LINGS_TEST
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CoordinateTransformNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
#endif
