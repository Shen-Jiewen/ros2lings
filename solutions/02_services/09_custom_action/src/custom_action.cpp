// 参考答案 — custom_action
// Countdown Action Server: 使用自定义 .action 文件实现倒数

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "ros2lings_24_custom_action/action/countdown.hpp"
#include <memory>
#include <thread>
#include <chrono>

using Countdown = ros2lings_24_custom_action::action::Countdown;
using GoalHandleCountdown = rclcpp_action::ServerGoalHandle<Countdown>;

class CountdownActionServer : public rclcpp::Node
{
public:
  CountdownActionServer() : Node("countdown_action_server")
  {
    using namespace std::placeholders;

    action_server_ = rclcpp_action::create_server<Countdown>(
      this,
      "countdown",
      std::bind(&CountdownActionServer::handle_goal, this, _1, _2),
      std::bind(&CountdownActionServer::handle_cancel, this, _1),
      std::bind(&CountdownActionServer::handle_accepted, this, _1));

    RCLCPP_INFO(get_logger(), "Countdown Action Server started");
  }

private:
  rclcpp_action::Server<Countdown>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Countdown::Goal> goal)
  {
    RCLCPP_INFO(get_logger(), "Received countdown goal: %d", goal->target_number);
    (void)uuid;
    if (goal->target_number <= 0) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleCountdown> goal_handle)
  {
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleCountdown> goal_handle)
  {
    std::thread{std::bind(&CountdownActionServer::execute, this, goal_handle)}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleCountdown> goal_handle)
  {
    RCLCPP_INFO(get_logger(), "Executing countdown");
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Countdown::Feedback>();
    auto result = std::make_shared<Countdown::Result>();

    for (int count = goal->target_number; count >= 0; --count) {
      if (goal_handle->is_canceling()) {
        result->final_count = count;
        goal_handle->canceled(result);
        RCLCPP_INFO(get_logger(), "Countdown canceled at %d", count);
        return;
      }
      feedback->current_count = count;
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(get_logger(), "Countdown: %d", count);
    }

    result->final_count = 0;
    goal_handle->succeed(result);
    RCLCPP_INFO(get_logger(), "Countdown complete");
  }
};

#ifndef ROS2LINGS_TEST
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CountdownActionServer>());
  rclcpp::shutdown();
  return 0;
}
#endif
