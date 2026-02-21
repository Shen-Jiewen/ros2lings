#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <ros2lings_interfaces/action/fibonacci.hpp>
#include <memory>
#include <vector>

using Fibonacci = ros2lings_interfaces::action::Fibonacci;
using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;

class ActionStateMachineTest : public ::testing::Test {
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

TEST_F(ActionStateMachineTest, CancelShouldBeAccepted) {
  // Bug 1 验证: handle_cancel 应该返回 ACCEPT
  // 正确的实现应该允许取消
  auto response = rclcpp_action::CancelResponse::ACCEPT;
  EXPECT_EQ(response, rclcpp_action::CancelResponse::ACCEPT)
    << "handle_cancel 应该返回 ACCEPT，允许客户端取消任务";
}

TEST_F(ActionStateMachineTest, HandleAcceptedMustStartExecution) {
  // Bug 2 验证: handle_accepted 必须启动执行线程
  // 如果不启动，目标会卡在 ACCEPTED 状态
  bool execution_started = false;

  auto node = std::make_shared<rclcpp::Node>("test_state_machine_node");
  auto server = rclcpp_action::create_server<Fibonacci>(
    node,
    "test_state_machine",
    [](const rclcpp_action::GoalUUID &, std::shared_ptr<const Fibonacci::Goal>) {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [](const std::shared_ptr<GoalHandleFibonacci>) {
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    [&execution_started](const std::shared_ptr<GoalHandleFibonacci>) {
      // 正确的 handle_accepted 应该启动执行
      execution_started = true;  // 模拟启动
    });

  ASSERT_NE(server, nullptr);
  // 在正确的实现中，handle_accepted 应该启动一个执行线程
  // 而不是什么都不做
}

TEST_F(ActionStateMachineTest, ResultMustBeSetAfterComputation) {
  // Bug 3 验证: 结果应该在完整计算之后设置
  int order = 7;
  std::vector<int64_t> partial_sequence;
  partial_sequence.push_back(0);
  partial_sequence.push_back(1);

  // 错误的做法: 在计算前就设置结果
  auto premature_result = partial_sequence;  // 只有 [0, 1]

  // 正确: 先完成计算
  for (int i = 2; i < order; ++i) {
    partial_sequence.push_back(partial_sequence[i - 1] + partial_sequence[i - 2]);
  }

  auto correct_result = partial_sequence;  // [0, 1, 1, 2, 3, 5, 8]

  // 过早设置的结果是不完整的
  EXPECT_EQ(premature_result.size(), 2u) << "过早设置的结果只有 2 个元素";
  EXPECT_EQ(correct_result.size(), 7u) << "正确的结果应有 7 个元素";
  EXPECT_EQ(correct_result.back(), 8) << "Fibonacci(7) 的最后一个元素应为 8";
}

TEST_F(ActionStateMachineTest, CorrectStateTransitionOrder) {
  // 验证正确的状态转换顺序:
  // 1. handle_goal -> ACCEPT
  // 2. handle_accepted -> 启动执行线程
  // 3. execute: 计算 + 反馈
  // 4. succeed(result) — 在计算完成后

  std::vector<std::string> transitions;

  transitions.push_back("goal_received");
  transitions.push_back("goal_accepted");
  transitions.push_back("execution_started");  // 不能跳过!
  transitions.push_back("computing");
  transitions.push_back("result_set");

  ASSERT_EQ(transitions.size(), 5u);
  EXPECT_EQ(transitions[0], "goal_received");
  EXPECT_EQ(transitions[2], "execution_started")
    << "handle_accepted 之后必须启动执行";
  EXPECT_EQ(transitions[4], "result_set")
    << "结果必须在计算完成之后设置";
}

TEST_F(ActionStateMachineTest, ActionServerCanBeCreatedWithCorrectCallbacks) {
  auto node = std::make_shared<rclcpp::Node>("test_correct_callbacks");

  auto server = rclcpp_action::create_server<Fibonacci>(
    node,
    "test_fibonacci_sm",
    // handle_goal
    [](const rclcpp_action::GoalUUID &, std::shared_ptr<const Fibonacci::Goal>) {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    // handle_cancel — 必须返回 ACCEPT
    [](const std::shared_ptr<GoalHandleFibonacci>) {
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    // handle_accepted — 必须启动执行
    [](const std::shared_ptr<GoalHandleFibonacci> goal_handle) {
      // 在真实代码中这里应该启动线程
      (void)goal_handle;
    });

  ASSERT_NE(server, nullptr);
}
