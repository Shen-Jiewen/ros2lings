#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <ros2lings_interfaces/action/fibonacci.hpp>
#include <memory>
#include <vector>
#include <chrono>

// Include student source directly so class definitions are visible in this translation unit
#include "../src/first_action_server.cpp"

using Fibonacci = ros2lings_interfaces::action::Fibonacci;
using GoalHandleFibonacci = rclcpp_action::ClientGoalHandle<Fibonacci>;
using namespace std::chrono_literals;

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

TEST_F(FirstActionServerTest, StudentNodeCanBeCreated) {
  auto server = std::make_shared<FibonacciActionServer>();
  ASSERT_NE(server, nullptr);
  EXPECT_EQ(std::string(server->get_name()), "fibonacci_action_server");
}

TEST_F(FirstActionServerTest, ActionServerAcceptsAndReturnsResult) {
  auto server_node = std::make_shared<FibonacciActionServer>();
  auto client_node = std::make_shared<rclcpp::Node>("test_action_client");
  auto client = rclcpp_action::create_client<Fibonacci>(client_node, "fibonacci");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server_node);
  executor.add_node(client_node);

  // Wait for action server to be available
  ASSERT_TRUE(client->wait_for_action_server(2s))
    << "Student's action server 'fibonacci' should be available";

  // Send a goal
  auto goal_msg = Fibonacci::Goal();
  goal_msg.order = 7;

  auto send_goal_future = client->async_send_goal(goal_msg);
  auto goal_status = executor.spin_until_future_complete(send_goal_future, 5s);
  ASSERT_EQ(goal_status, rclcpp::FutureReturnCode::SUCCESS);

  auto goal_handle = send_goal_future.get();
  ASSERT_NE(goal_handle, nullptr) << "Goal should be accepted";

  // Wait for result
  auto result_future = client->async_get_result(goal_handle);
  auto result_status = executor.spin_until_future_complete(result_future, 5s);
  ASSERT_EQ(result_status, rclcpp::FutureReturnCode::SUCCESS);

  auto wrapped_result = result_future.get();
  ASSERT_EQ(wrapped_result.code, rclcpp_action::ResultCode::SUCCEEDED)
    << "Goal should succeed";

  // Verify the Fibonacci sequence: 0, 1, 1, 2, 3, 5, 8
  auto & seq = wrapped_result.result->sequence;
  ASSERT_EQ(seq.size(), 7u);
  EXPECT_EQ(seq[0], 0);
  EXPECT_EQ(seq[1], 1);
  EXPECT_EQ(seq[2], 1);
  EXPECT_EQ(seq[3], 2);
  EXPECT_EQ(seq[4], 3);
  EXPECT_EQ(seq[5], 5);
  EXPECT_EQ(seq[6], 8);
}
