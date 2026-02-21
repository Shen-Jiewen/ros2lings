// I AM NOT DONE
//
// 练习: custom_action
// 模块: 02 - Services & Actions
// 难度: ★★☆☆☆
//
// 学习目标:
//   理解如何使用自定义 .action 文件定义的 Action 接口，
//   实现一个 Countdown Action Server。
//
// 说明:
//   你需要补全下面的代码，使用自定义的 Countdown action。
//   该 action 接收 target_number（目标数），倒数到 0，
//   通过 feedback 报告 current_count，最终返回 final_count。
//   请根据 TODO 提示完成实现。
//
// 步骤:
//   1. 实现 handle_goal 中的条件判断（拒绝非正数目标）
//   2. 实现 execute 中的倒数循环和反馈发布
//   3. 修复完成后，删除文件顶部的 "// I AM NOT DONE"

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
    // TODO: 如果 target_number <= 0，返回 REJECT
    // TODO: 否则返回 ACCEPT_AND_EXECUTE
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

    // TODO: 实现倒数循环
    // for (int count = goal->target_number; count >= 0; --count) {
    //   // 检查取消
    //   if (goal_handle->is_canceling()) {
    //     result->final_count = count;
    //     goal_handle->canceled(result);
    //     RCLCPP_INFO(get_logger(), "Countdown canceled at %d", count);
    //     return;
    //   }
    //   // 发布反馈
    //   feedback->current_count = count;
    //   goal_handle->publish_feedback(feedback);
    //   RCLCPP_INFO(get_logger(), "Countdown: %d", count);
    // }

    // TODO: 设置结果并标记成功
    // result->final_count = 0;
    // goal_handle->succeed(result);
    RCLCPP_INFO(get_logger(), "Countdown complete");
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CountdownActionServer>());
  rclcpp::shutdown();
  return 0;
}
