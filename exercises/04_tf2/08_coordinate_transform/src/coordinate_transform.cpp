// I AM NOT DONE
//
// 练习: coordinate_transform
// 模块: 04 - TF2 Transforms
// 难度: ★★★☆☆
//
// 学习目标:
//   掌握 tf2::doTransform 的使用方法，将一个点从一个坐标帧变换到另一个坐标帧。
//   理解 lookupTransform 和 doTransform 的配合使用。
//
// 说明:
//   下面的节点需要将一个在 "sensor_link" 帧中的点变换到 "map" 帧中。
//   你需要补全 4 个 TODO 来完成这个功能。
//
// 步骤:
//   1. 创建 tf2_ros::Buffer 和 tf2_ros::TransformListener（TODO 1）
//   2. 创建一个 PointStamped 点，帧为 "sensor_link"，坐标 (1.0, 0.0, 0.0)（TODO 2）
//   3. 使用 lookupTransform 获取 "sensor_link" 到 "map" 的变换（TODO 3）
//   4. 使用 tf2::doTransform 将点变换到 "map" 帧（TODO 4）
//   5. 完成后，删除文件顶部的 "// I AM NOT DONE"

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
    // TODO 1: 创建 tf2_ros::Buffer 和 tf2_ros::TransformListener
    // 提示:
    //   tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    //   tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

    // 创建定时器，每秒尝试一次坐标变换
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&CoordinateTransformNode::timer_callback, this));
  }

private:
  void timer_callback()
  {
    // TODO 2: 创建一个 PointStamped 点
    // 提示:
    //   geometry_msgs::msg::PointStamped point_in;
    //   point_in.header.frame_id = "sensor_link";
    //   point_in.header.stamp = this->now();
    //   point_in.point.x = 1.0;
    //   point_in.point.y = 0.0;
    //   point_in.point.z = 0.0;

    try {
      // TODO 3: 使用 lookupTransform 获取从 "sensor_link" 到 "map" 的变换
      // 提示: lookupTransform 的参数顺序是 (target_frame, source_frame, time)
      //   auto transform = tf2_buffer_->lookupTransform(
      //     "map", "sensor_link", tf2::TimePointZero);

      // TODO 4: 使用 tf2::doTransform 将 point_in 变换到 map 帧
      // 提示:
      //   geometry_msgs::msg::PointStamped point_out;
      //   tf2::doTransform(point_in, point_out, transform);

      // 打印变换后的点坐标（完成 TODO 4 后取消注释）
      // RCLCPP_INFO(this->get_logger(),
      //   "sensor_link 中的点 (%.1f, %.1f, %.1f) -> map 中的点 (%.1f, %.1f, %.1f)",
      //   point_in.point.x, point_in.point.y, point_in.point.z,
      //   point_out.point.x, point_out.point.y, point_out.point.z);

    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "坐标变换失败: %s", ex.what());
    }
  }

  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CoordinateTransformNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
