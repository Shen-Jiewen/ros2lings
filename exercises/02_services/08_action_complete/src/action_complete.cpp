// I AM NOT DONE
//
// 练习: action_complete
// 模块: 02 - Services & Actions
// 难度: ★★★☆☆
//
// 学习目标:
//   实现一个完整的 Action Server，包括 Fibonacci 计算、
//   反馈发布和取消请求处理。
//
// 说明:
//   框架代码已经给出，你需要补全 execute 方法中的核心逻辑。
//   请根据 TODO 提示完成实现。
//
// 步骤:
//   1. 在 execute 方法的 for 循环中实现 Fibonacci 计算
//   2. 在每步计算后发布反馈
//   3. 在循环中检查取消请求
//   4. 循环结束后设置结果并调用 succeed
//   5. 修复完成后，删除文件顶部的 "// I AM NOT DONE"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <ros2lings_interfaces/action/fibonacci.hpp>
#include <memory>
#include <thread>
#include <chrono>

using Fibonacci = ros2lings_interfaces::action::Fibonacci;
using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;

class FibonacciCompleteServer : public rclcpp::Node
{
public:
  FibonacciCompleteServer() : Node("fibonacci_complete_server")
  {
    using namespace std::placeholders;

    action_server_ = rclcpp_action::create_server<Fibonacci>(
      this,
      "fibonacci",
      std::bind(&FibonacciCompleteServer::handle_goal, this, _1, _2),
      std::bind(&FibonacciCompleteServer::handle_cancel, this, _1),
      std::bind(&FibonacciCompleteServer::handle_accepted, this, _1));

    RCLCPP_INFO(get_logger(), "Fibonacci Complete Server started");
  }

private:
  rclcpp_action::Server<Fibonacci>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Fibonacci::Goal> goal)
  {
    RCLCPP_INFO(get_logger(), "Received goal request with order %d", goal->order);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    RCLCPP_INFO(get_logger(), "Received cancel request");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    std::thread{std::bind(&FibonacciCompleteServer::execute, this, goal_handle)}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    RCLCPP_INFO(get_logger(), "Executing goal");
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Fibonacci::Feedback>();
    auto result = std::make_shared<Fibonacci::Result>();

    auto & partial_sequence = feedback->partial_sequence;
    partial_sequence.push_back(0);
    partial_sequence.push_back(1);

    for (int i = 2; i < goal->order; ++i) {
      // TODO: 检查取消请求
      // if (goal_handle->is_canceling()) {
      //   result->sequence = partial_sequence;
      //   goal_handle->canceled(result);
      //   RCLCPP_INFO(get_logger(), "Goal canceled");
      //   return;
      // }

      // TODO: 计算下一个 Fibonacci 数
      // partial_sequence.push_back(partial_sequence[i - 1] + partial_sequence[i - 2]);

      // TODO: 发布反馈
      // goal_handle->publish_feedback(feedback);
    }

    // TODO: 设置结果并标记成功
    // result->sequence = partial_sequence;
    // goal_handle->succeed(result);
    RCLCPP_INFO(get_logger(), "Goal execution complete");
  }
};

#ifndef ROS2LINGS_TEST
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FibonacciCompleteServer>());
  rclcpp::shutdown();
  return 0;
}
#endif
