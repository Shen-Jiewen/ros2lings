#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <ros2lings_interfaces/action/fibonacci.hpp>
#include <memory>
#include <vector>

using Fibonacci = ros2lings_interfaces::action::Fibonacci;
using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;

class FirstActionServerTest : public ::testing::Test {
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

TEST_F(FirstActionServerTest, FibonacciComputationIsCorrect) {
  // 测试 Fibonacci 计算逻辑（不依赖 Action Server）
  int order = 7;
  std::vector<int64_t> sequence;
  sequence.push_back(0);
  sequence.push_back(1);
  for (int i = 2; i < order; ++i) {
    sequence.push_back(sequence[i - 1] + sequence[i - 2]);
  }

  // Fibonacci(7): 0, 1, 1, 2, 3, 5, 8
  ASSERT_EQ(sequence.size(), 7u);
  EXPECT_EQ(sequence[0], 0);
  EXPECT_EQ(sequence[1], 1);
  EXPECT_EQ(sequence[2], 1);
  EXPECT_EQ(sequence[3], 2);
  EXPECT_EQ(sequence[4], 3);
  EXPECT_EQ(sequence[5], 5);
  EXPECT_EQ(sequence[6], 8);
}

TEST_F(FirstActionServerTest, GoalResponseTypes) {
  // 测试 GoalResponse 枚举值存在
  auto accept = rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  auto reject = rclcpp_action::GoalResponse::REJECT;
  EXPECT_NE(static_cast<int>(accept), static_cast<int>(reject));
}

TEST_F(FirstActionServerTest, CancelResponseTypes) {
  // 测试 CancelResponse 枚举值存在
  auto accept = rclcpp_action::CancelResponse::ACCEPT;
  auto reject = rclcpp_action::CancelResponse::REJECT;
  EXPECT_NE(static_cast<int>(accept), static_cast<int>(reject));
}

TEST_F(FirstActionServerTest, CanCreateActionServer) {
  auto node = std::make_shared<rclcpp::Node>("test_action_server_node");

  auto server = rclcpp_action::create_server<Fibonacci>(
    node,
    "test_fibonacci",
    [](const rclcpp_action::GoalUUID &, std::shared_ptr<const Fibonacci::Goal>) {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [](const std::shared_ptr<GoalHandleFibonacci>) {
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    [](const std::shared_ptr<GoalHandleFibonacci>) {
      // handle_accepted
    });

  ASSERT_NE(server, nullptr);
}

TEST_F(FirstActionServerTest, FibonacciMessageTypes) {
  // 测试 Fibonacci Action 消息类型的结构
  auto goal = Fibonacci::Goal();
  goal.order = 5;
  EXPECT_EQ(goal.order, 5);

  auto result = Fibonacci::Result();
  result.sequence.push_back(0);
  result.sequence.push_back(1);
  EXPECT_EQ(result.sequence.size(), 2u);

  auto feedback = Fibonacci::Feedback();
  feedback.partial_sequence.push_back(0);
  EXPECT_EQ(feedback.partial_sequence.size(), 1u);
}
