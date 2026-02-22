// I AM NOT DONE
//
// 练习: tf_listener
// 模块: 04 - TF2 Transforms
// 难度: ★★☆☆☆
//
// 学习目标:
//   理解 tf2_ros::Buffer 和 tf2_ros::TransformListener 的配合使用，
//   掌握 lookupTransform 的参数含义和异常处理。
//
// 说明:
//   下面的节点试图查询从 "base_link" 到 "sensor_link" 的坐标变换，
//   但有 3 个错误需要你修复。
//
// 步骤:
//   1. 创建 Buffer 时传入节点时钟 — 应该用 this->get_clock()
//   2. 修复 lookupTransform 的帧参数顺序 — 源帧和目标帧反了
//   3. 给 lookupTransform 添加 try-catch 异常处理
//   4. 修复完成后，删除文件顶部的 "// I AM NOT DONE"

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class TfListenerNode : public rclcpp::Node
{
public:
  TfListenerNode() : Node("tf_listener_node")
  {
    // BUG 1: 创建 Buffer 时没有传入时钟
    // 正确做法: tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>();

    // 创建 TransformListener，它会自动订阅 /tf 和 /tf_static
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // 创建定时器，每 500ms 查询一次变换
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&TfListenerNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "TF 监听器已启动，等待变换数据...");
  }

  // 访问函数（供测试使用）
  std::string get_target_frame() const { return target_frame_; }
  std::string get_source_frame() const { return source_frame_; }
  std::shared_ptr<tf2_ros::Buffer> get_buffer() const { return tf_buffer_; }

private:
  void timer_callback()
  {
    // BUG 2: lookupTransform 的参数顺序是 (target_frame, source_frame, time)
    // 这里写反了：应该是 lookupTransform("base_link", "sensor_link", ...)
    // 表示"查询 sensor_link 在 base_link 坐标系下的位姿"

    // BUG 3: lookupTransform 可能抛出异常（如变换还不可用），
    // 必须用 try-catch 包裹
    geometry_msgs::msg::TransformStamped t;
    t = tf_buffer_->lookupTransform(
      "sensor_link",  // 应该是 target_frame: "base_link"
      "base_link",    // 应该是 source_frame: "sensor_link"
      tf2::TimePointZero);

    RCLCPP_INFO(this->get_logger(),
      "变换 base_link -> sensor_link: [%.2f, %.2f, %.2f]",
      t.transform.translation.x,
      t.transform.translation.y,
      t.transform.translation.z);
  }

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string target_frame_ = "base_link";
  std::string source_frame_ = "sensor_link";
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TfListenerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
