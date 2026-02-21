#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "ros2lings_24_custom_action/action/countdown.hpp"
#include <memory>

using Countdown = ros2lings_24_custom_action::action::Countdown;

class CustomActionTest : public ::testing::Test {
protected:
  void SetUp() override {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }
  void TearDown() override {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

TEST_F(CustomActionTest, GoalMessageHasTargetNumber) {
  auto goal = Countdown::Goal();
  goal.target_number = 10;
  EXPECT_EQ(goal.target_number, 10);
}

TEST_F(CustomActionTest, ResultMessageHasFinalCount) {
  auto result = Countdown::Result();
  result.final_count = 0;
  EXPECT_EQ(result.final_count, 0);
}

TEST_F(CustomActionTest, FeedbackMessageHasCurrentCount) {
  auto feedback = Countdown::Feedback();
  feedback.current_count = 5;
  EXPECT_EQ(feedback.current_count, 5);
}

TEST_F(CustomActionTest, CountdownLogic) {
  // 模拟倒数逻辑
  int target = 5;
  std::vector<int> feedback_values;

  for (int count = target; count >= 0; --count) {
    feedback_values.push_back(count);
  }

  // 验证反馈序列: 5, 4, 3, 2, 1, 0
  ASSERT_EQ(feedback_values.size(), 6u);
  EXPECT_EQ(feedback_values[0], 5);
  EXPECT_EQ(feedback_values[5], 0);
}

TEST_F(CustomActionTest, CanCreateCountdownActionServer) {
  using GoalHandleCountdown = rclcpp_action::ServerGoalHandle<Countdown>;

  auto node = std::make_shared<rclcpp::Node>("test_countdown_node");
  auto server = rclcpp_action::create_server<Countdown>(
    node,
    "test_countdown",
    [](const rclcpp_action::GoalUUID &, std::shared_ptr<const Countdown::Goal>) {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [](const std::shared_ptr<GoalHandleCountdown>) {
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    [](const std::shared_ptr<GoalHandleCountdown>) {});

  ASSERT_NE(server, nullptr);
}
