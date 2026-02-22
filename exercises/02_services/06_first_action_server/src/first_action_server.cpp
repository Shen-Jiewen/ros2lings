// I AM NOT DONE
//
// 练习: first_action_server
// 模块: 02 - Services & Actions
// 难度: ★★★☆☆
//
// 学习目标:
//   理解 ROS2 Action Server 的回调结构：handle_goal、handle_cancel、
//   handle_accepted 和 execute 的签名和职责。
//
// 说明:
//   下面的代码尝试创建一个 Fibonacci Action Server，但代码中有四个错误。
//   你需要找到并修复它们。
//
// 步骤:
//   1. 修复 handle_goal 的返回类型（不是 bool）
//   2. 修复 handle_cancel 的参数类型（不是原始指针）
//   3. 在 handle_accepted 中启动执行线程
//   4. 在 execute 中添加 succeed 调用来设置结果
//   5. 修复完成后，删除文件顶部的 "// I AM NOT DONE"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <ros2lings_interfaces/action/fibonacci.hpp>
#include <memory>
#include <thread>

using Fibonacci = ros2lings_interfaces::action::Fibonacci;
using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;

class FibonacciActionServer : public rclcpp::Node
{
public:
  FibonacciActionServer() : Node("fibonacci_action_server")
  {
    using namespace std::placeholders;

    action_server_ = rclcpp_action::create_server<Fibonacci>(
      this,
      "fibonacci",
      std::bind(&FibonacciActionServer::handle_goal, this, _1, _2),
      std::bind(&FibonacciActionServer::handle_cancel, this, _1),
      std::bind(&FibonacciActionServer::handle_accepted, this, _1));

    RCLCPP_INFO(get_logger(), "Fibonacci Action Server started");
  }

private:
  rclcpp_action::Server<Fibonacci>::SharedPtr action_server_;

  // BUG 1: 返回类型应该是 rclcpp_action::GoalResponse，不是 bool
  bool handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Fibonacci::Goal> goal)
  {
    RCLCPP_INFO(get_logger(), "Received goal request with order %d", goal->order);
    (void)uuid;
    return true;
  }

  // BUG 2: 参数应该是 shared_ptr，不是原始指针
  rclcpp_action::CancelResponse handle_cancel(
    const GoalHandleFibonacci * goal_handle)
  {
    RCLCPP_INFO(get_logger(), "Received cancel request");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  // BUG 3: 需要启动执行线程
  void handle_accepted(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    RCLCPP_INFO(get_logger(), "Goal accepted");
    // 缺少线程启动
  }

  void execute(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    RCLCPP_INFO(get_logger(), "Executing goal");
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<Fibonacci::Result>();

    auto & sequence = result->sequence;
    sequence.push_back(0);
    sequence.push_back(1);
    for (int i = 2; i < goal->order; ++i) {
      sequence.push_back(sequence[i - 1] + sequence[i - 2]);
    }

    // BUG 4: 缺少 goal_handle->succeed(result) 调用
    RCLCPP_INFO(get_logger(), "Goal finished");
  }
};

#ifndef ROS2LINGS_TEST
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FibonacciActionServer>());
  rclcpp::shutdown();
  return 0;
}
#endif
