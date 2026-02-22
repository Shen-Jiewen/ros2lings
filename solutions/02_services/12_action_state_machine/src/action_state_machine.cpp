// 参考答案 — action_state_machine
// 修复后的 Action 状态机: 正确的状态转换

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <ros2lings_interfaces/action/fibonacci.hpp>
#include <memory>
#include <thread>
#include <chrono>

using Fibonacci = ros2lings_interfaces::action::Fibonacci;
using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;
using namespace std::chrono_literals;

class FibonacciStateMachine : public rclcpp::Node
{
public:
  FibonacciStateMachine() : Node("fibonacci_state_machine")
  {
    using namespace std::placeholders;

    action_server_ = rclcpp_action::create_server<Fibonacci>(
      this,
      "fibonacci",
      std::bind(&FibonacciStateMachine::handle_goal, this, _1, _2),
      std::bind(&FibonacciStateMachine::handle_cancel, this, _1),
      std::bind(&FibonacciStateMachine::handle_accepted, this, _1));

    RCLCPP_INFO(get_logger(), "Fibonacci State Machine Server started");
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

  // FIX 1: 返回 ACCEPT 允许取消
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    RCLCPP_INFO(get_logger(), "Received cancel request");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  // FIX 2: 启动执行线程
  void handle_accepted(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    RCLCPP_INFO(get_logger(), "Goal accepted, starting execution");
    std::thread{std::bind(&FibonacciStateMachine::execute, this, goal_handle)}.detach();
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

    // FIX 3: 先完成计算，再设置结果
    for (int i = 2; i < goal->order; ++i) {
      if (goal_handle->is_canceling()) {
        result->sequence = partial_sequence;
        goal_handle->canceled(result);
        RCLCPP_INFO(get_logger(), "Goal canceled");
        return;
      }
      partial_sequence.push_back(partial_sequence[i - 1] + partial_sequence[i - 2]);
      goal_handle->publish_feedback(feedback);
    }

    // 计算完成后才设置结果
    result->sequence = partial_sequence;
    goal_handle->succeed(result);
    RCLCPP_INFO(get_logger(), "Goal execution complete");
  }
};

#ifndef ROS2LINGS_TEST
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FibonacciStateMachine>());
  rclcpp::shutdown();
  return 0;
}
#endif
