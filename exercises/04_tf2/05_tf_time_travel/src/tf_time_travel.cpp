// I AM NOT DONE
//
// 练习: tf_time_travel
// 模块: 04 - TF2 Transforms
// 难度: ★★★☆☆
//
// 学习目标:
//   理解 TF2 的时间戳查询机制，掌握 tf2::TimePointZero 的作用，
//   了解 lookupTransform 的 timeout 参数和正确的异常处理方式。
//
// 说明:
//   下面的节点试图通过 lookupTransform 查询 "world" 到 "robot" 的变换，
//   但有 3 个 BUG 需要你修复。代码可以编译，但运行时行为不正确。
//
// 步骤:
//   1. 修复 lookupTransform 的时间戳参数 — 不应使用未来时间
//   2. 修复 timeout 参数 — 不应设为 0 秒
//   3. 修复 catch 块的异常类型 — 应该捕获 tf2 的异常
//   4. 修复完成后，删除文件顶部的 "// I AM NOT DONE"

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <chrono>

using namespace std::chrono_literals;

class TfTimeTravelNode : public rclcpp::Node
{
public:
  TfTimeTravelNode() : Node("tf_time_travel_node")
  {
    // 创建 Buffer 和 TransformListener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // 创建定时器，每 500ms 查询一次变换
    timer_ = this->create_wall_timer(
      500ms,
      std::bind(&TfTimeTravelNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "TF 时间旅行节点已启动");
  }

  // 访问函数（供测试使用）
  std::shared_ptr<tf2_ros::Buffer> get_buffer() const { return tf_buffer_; }

#ifdef ROS2LINGS_TEST
  // Test-only: track whether timer_callback successfully looked up a transform
  bool get_lookup_succeeded() const { return lookup_succeeded_; }
  int get_callback_count() const { return callback_count_; }
#endif

private:
  void timer_callback()
  {
    // BUG 1: 使用了一个"未来"的时间戳来查询变换
    // 这会导致查询失败，因为 buffer 中不可能有尚未发生的变换数据
    // 正确做法: 使用 tf2::TimePointZero 来获取最新可用的变换
    auto future_time = std::chrono::system_clock::now() + std::chrono::seconds(10);
    auto query_time = tf2::TimePoint(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        future_time.time_since_epoch()));

    // BUG 2: timeout 设置为 0 秒，不会等待变换变为可用
    // 当变换还没有被发布时，lookupTransform 会立即失败
    // 正确做法: 设置一个合理的超时时间，如 1 秒
    auto timeout = rclcpp::Duration::from_seconds(0.0);

    try {
      geometry_msgs::msg::TransformStamped t;
      t = tf_buffer_->lookupTransform(
        "world",
        "robot",
        query_time,        // BUG 1: 应该用 tf2::TimePointZero
        timeout.to_chrono<std::chrono::nanoseconds>());  // BUG 2: 应该用合理的超时

      RCLCPP_INFO(this->get_logger(),
        "变换 world -> robot: [%.2f, %.2f, %.2f]",
        t.transform.translation.x,
        t.transform.translation.y,
        t.transform.translation.z);
#ifdef ROS2LINGS_TEST
      lookup_succeeded_ = true;
#endif

    // BUG 3: 捕获了 std::runtime_error 而不是 tf2::TransformException
    // tf2 的异常（LookupException, ExtrapolationException 等）
    // 都继承自 tf2::TransformException，不一定继承自 std::runtime_error
    } catch (const std::runtime_error & ex) {
      RCLCPP_WARN(this->get_logger(), "查询失败: %s", ex.what());
    }
#ifdef ROS2LINGS_TEST
    callback_count_++;
#endif
  }

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
#ifdef ROS2LINGS_TEST
  bool lookup_succeeded_ = false;
  int callback_count_ = 0;
#endif
};

#ifndef ROS2LINGS_TEST
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TfTimeTravelNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
#endif
