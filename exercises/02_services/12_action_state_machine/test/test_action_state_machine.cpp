#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <ros2lings_interfaces/action/fibonacci.hpp>
#include <memory>
#include <vector>
#include <chrono>

// Student source is compiled together with this test via CMakeLists.txt.
// The student's FibonacciStateMachine class is available because main()
// is guarded by #ifndef ROS2LINGS_TEST.

using Fibonacci = ros2lings_interfaces::action::Fibonacci;
using GoalHandleFibonacci = rclcpp_action::ClientGoalHandle<Fibonacci>;
using namespace std::chrono_literals;

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

TEST_F(ActionStateMachineTest, StudentNodeCanBeCreated) {
  auto server = std::make_shared<FibonacciStateMachine>();
  ASSERT_NE(server, nullptr);
  EXPECT_EQ(std::string(server->get_name()), "fibonacci_state_machine");
}

TEST_F(ActionStateMachineTest, GoalExecutesAndReturnsCompleteResult) {
  // Bug 2: handle_accepted must start execution thread
  // Bug 3: result must be set after computation, not before
  auto server_node = std::make_shared<FibonacciStateMachine>();
  auto client_node = std::make_shared<rclcpp::Node>("test_sm_client");
  auto client = rclcpp_action::create_client<Fibonacci>(client_node, "fibonacci");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server_node);
  executor.add_node(client_node);

  ASSERT_TRUE(client->wait_for_action_server(2s))
    << "Student's action server 'fibonacci' should be available";

  auto goal_msg = Fibonacci::Goal();
  goal_msg.order = 7;

  auto send_goal_future = client->async_send_goal(goal_msg);
  auto goal_status = executor.spin_until_future_complete(send_goal_future, 5s);
  ASSERT_EQ(goal_status, rclcpp::FutureReturnCode::SUCCESS);

  auto goal_handle = send_goal_future.get();
  ASSERT_NE(goal_handle, nullptr) << "Goal should be accepted";

  auto result_future = client->async_get_result(goal_handle);
  auto result_status = executor.spin_until_future_complete(result_future, 5s);
  ASSERT_EQ(result_status, rclcpp::FutureReturnCode::SUCCESS)
    << "Goal should complete (handle_accepted must start execution thread)";

  auto wrapped_result = result_future.get();
  ASSERT_EQ(wrapped_result.code, rclcpp_action::ResultCode::SUCCEEDED)
    << "Goal should succeed";

  // Fibonacci(7): 0, 1, 1, 2, 3, 5, 8
  // Bug 3 verification: result must contain the COMPLETE sequence
  auto & seq = wrapped_result.result->sequence;
  ASSERT_EQ(seq.size(), 7u)
    << "Result must contain all 7 elements (succeed must be called after computation)";
  EXPECT_EQ(seq[0], 0);
  EXPECT_EQ(seq[1], 1);
  EXPECT_EQ(seq[6], 8);
}

TEST_F(ActionStateMachineTest, CancelIsAccepted) {
  // Bug 1: handle_cancel must return ACCEPT, not REJECT
  auto server_node = std::make_shared<FibonacciStateMachine>();
  auto client_node = std::make_shared<rclcpp::Node>("test_cancel_client");
  auto client = rclcpp_action::create_client<Fibonacci>(client_node, "fibonacci");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server_node);
  executor.add_node(client_node);

  ASSERT_TRUE(client->wait_for_action_server(2s));

  // Send a goal with a large order to give time to cancel
  auto goal_msg = Fibonacci::Goal();
  goal_msg.order = 1000000;

  auto send_goal_future = client->async_send_goal(goal_msg);
  auto goal_status = executor.spin_until_future_complete(send_goal_future, 5s);
  ASSERT_EQ(goal_status, rclcpp::FutureReturnCode::SUCCESS);

  auto goal_handle = send_goal_future.get();
  ASSERT_NE(goal_handle, nullptr);

  // Request cancellation
  auto cancel_future = client->async_cancel_goal(goal_handle);
  auto cancel_status = executor.spin_until_future_complete(cancel_future, 5s);
  ASSERT_EQ(cancel_status, rclcpp::FutureReturnCode::SUCCESS);

  auto cancel_response = cancel_future.get();
  // If handle_cancel returns ACCEPT, cancellation should succeed
  EXPECT_FALSE(cancel_response->goals_canceling.empty())
    << "handle_cancel should return ACCEPT to allow cancellation";
}
