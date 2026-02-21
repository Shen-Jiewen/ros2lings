#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <ros2lings_interfaces/action/fibonacci.hpp>
#include <memory>
#include <vector>
#include <chrono>

using Fibonacci = ros2lings_interfaces::action::Fibonacci;
using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;

class ActionCompleteTest : public ::testing::Test {
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

TEST_F(ActionCompleteTest, FibonacciWithFeedbackLogic) {
  // 模拟 execute 中的 Fibonacci 计算和反馈逻辑
  int order = 8;
  std::vector<int64_t> partial_sequence;
  partial_sequence.push_back(0);
  partial_sequence.push_back(1);

  std::vector<size_t> feedback_lengths;

  for (int i = 2; i < order; ++i) {
    partial_sequence.push_back(partial_sequence[i - 1] + partial_sequence[i - 2]);
    feedback_lengths.push_back(partial_sequence.size());
  }

  // 验证最终序列
  ASSERT_EQ(partial_sequence.size(), 8u);
  EXPECT_EQ(partial_sequence[0], 0);
  EXPECT_EQ(partial_sequence[1], 1);
  EXPECT_EQ(partial_sequence[7], 13);

  // 验证反馈在每步都被"发布"
  EXPECT_EQ(feedback_lengths.size(), 6u);
  EXPECT_EQ(feedback_lengths[0], 3u);  // 第一次反馈: 3 个元素
  EXPECT_EQ(feedback_lengths[5], 8u);  // 最后一次反馈: 8 个元素
}

TEST_F(ActionCompleteTest, CancelLogicSimulation) {
  // 模拟取消时的行为
  int order = 10;
  int cancel_at = 5;
  bool was_canceled = false;

  std::vector<int64_t> partial_sequence;
  partial_sequence.push_back(0);
  partial_sequence.push_back(1);

  for (int i = 2; i < order; ++i) {
    // 模拟 is_canceling()
    if (i == cancel_at) {
      was_canceled = true;
      break;
    }
    partial_sequence.push_back(partial_sequence[i - 1] + partial_sequence[i - 2]);
  }

  EXPECT_TRUE(was_canceled);
  // 取消时序列应该不完整
  EXPECT_LT(partial_sequence.size(), static_cast<size_t>(order));
}

TEST_F(ActionCompleteTest, FeedbackMessageCanBePublished) {
  auto feedback = std::make_shared<Fibonacci::Feedback>();
  feedback->partial_sequence.push_back(0);
  feedback->partial_sequence.push_back(1);
  feedback->partial_sequence.push_back(1);

  EXPECT_EQ(feedback->partial_sequence.size(), 3u);
  EXPECT_EQ(feedback->partial_sequence[2], 1);
}

TEST_F(ActionCompleteTest, ResultCanBeSet) {
  auto result = std::make_shared<Fibonacci::Result>();
  result->sequence = {0, 1, 1, 2, 3, 5, 8, 13};

  EXPECT_EQ(result->sequence.size(), 8u);
  EXPECT_EQ(result->sequence.back(), 13);
}
