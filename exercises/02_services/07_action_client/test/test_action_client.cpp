#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <ros2lings_interfaces/action/fibonacci.hpp>
#include <memory>

using Fibonacci = ros2lings_interfaces::action::Fibonacci;
using GoalHandleFibonacci = rclcpp_action::ClientGoalHandle<Fibonacci>;

class ActionClientTest : public ::testing::Test {
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

TEST_F(ActionClientTest, CanCreateActionClient) {
  auto node = std::make_shared<rclcpp::Node>("test_client_node");
  auto client = rclcpp_action::create_client<Fibonacci>(node, "test_fibonacci");
  ASSERT_NE(client, nullptr);
}

TEST_F(ActionClientTest, SendGoalOptionsHasCallbacks) {
  // 验证 SendGoalOptions 结构体包含所需的回调字段
  auto options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();

  // 设置所有三个回调
  options.goal_response_callback = [](const GoalHandleFibonacci::SharedPtr &) {};
  options.feedback_callback = [](GoalHandleFibonacci::SharedPtr,
    const std::shared_ptr<const Fibonacci::Feedback>) {};
  options.result_callback = [](const GoalHandleFibonacci::WrappedResult &) {};

  // 如果能编译通过，说明回调字段存在
  SUCCEED();
}

TEST_F(ActionClientTest, GoalMessageCanBeCreated) {
  auto goal = Fibonacci::Goal();
  goal.order = 10;
  EXPECT_EQ(goal.order, 10);
}

TEST_F(ActionClientTest, ResultCodeEnumValues) {
  // 验证 ResultCode 枚举值
  EXPECT_NE(
    static_cast<int>(rclcpp_action::ResultCode::SUCCEEDED),
    static_cast<int>(rclcpp_action::ResultCode::ABORTED));
  EXPECT_NE(
    static_cast<int>(rclcpp_action::ResultCode::SUCCEEDED),
    static_cast<int>(rclcpp_action::ResultCode::CANCELED));
}

TEST_F(ActionClientTest, FeedbackMessageStructure) {
  auto feedback = Fibonacci::Feedback();
  feedback.partial_sequence.push_back(0);
  feedback.partial_sequence.push_back(1);
  feedback.partial_sequence.push_back(1);
  EXPECT_EQ(feedback.partial_sequence.size(), 3u);
  EXPECT_EQ(feedback.partial_sequence[2], 1);
}
