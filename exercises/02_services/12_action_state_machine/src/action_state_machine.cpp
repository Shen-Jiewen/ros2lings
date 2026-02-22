// I AM NOT DONE
//
// 练习: action_state_machine
// 模块: 02 - Services & Actions
// 难度: ★★★☆☆
//
// 学习目标:
//   理解 ROS2 Action 的状态机生命周期，找出导致状态转换错误的
//   逻辑 bug。
//
// 说明:
//   下面的代码实现了一个 Fibonacci Action Server，但在状态管理方面
//   有三个逻辑错误，导致 Action 无法正确运行。你需要通过理解
//   Action 状态机来找出并修复这些 bug。
//
// 步骤:
//   1. 修复 handle_cancel 的返回值（让客户端能够取消任务）
//   2. 修复 handle_accepted（目标被接受后必须开始执行）
//   3. 修复 execute 中的执行顺序（结果应该在计算完成后设置）
//   4. 修复完成后，删除文件顶部的 "// I AM NOT DONE"

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

  // BUG 1: 返回 REJECT 导致客户端无法取消任务
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    RCLCPP_INFO(get_logger(), "Received cancel request");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::REJECT;
  }

  // BUG 2: 没有启动执行线程，目标卡在 ACCEPTED 状态
  void handle_accepted(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    RCLCPP_INFO(get_logger(), "Goal accepted, starting execution");
    // 缺少线程启动，目标永远不会执行
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

    // BUG 3: 在计算完成前就调用 succeed，导致结果不完整
    result->sequence = partial_sequence;
    goal_handle->succeed(result);
    RCLCPP_INFO(get_logger(), "Goal execution complete");

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
