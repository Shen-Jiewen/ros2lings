#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <ros2lings_interfaces/action/fibonacci.hpp>
#include <memory>
#include <vector>
#include <chrono>
#include <mutex>

// Student source is compiled together with this test via CMakeLists.txt.
// The student's FibonacciCompleteServer class is available because main()
// is guarded by #ifndef ROS2LINGS_TEST.

using Fibonacci = ros2lings_interfaces::action::Fibonacci;
using GoalHandleFibonacci = rclcpp_action::ClientGoalHandle<Fibonacci>;
using namespace std::chrono_literals;

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

TEST_F(ActionCompleteTest, StudentNodeCanBeCreated) {
  auto server = std::make_shared<FibonacciCompleteServer>();
  ASSERT_NE(server, nullptr);
  EXPECT_EQ(std::string(server->get_name()), "fibonacci_complete_server");
}

TEST_F(ActionCompleteTest, ActionServerReturnsCorrectResult) {
  auto server_node = std::make_shared<FibonacciCompleteServer>();
  auto client_node = std::make_shared<rclcpp::Node>("test_complete_client");
  auto client = rclcpp_action::create_client<Fibonacci>(client_node, "fibonacci");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server_node);
  executor.add_node(client_node);

  ASSERT_TRUE(client->wait_for_action_server(2s))
    << "Student's action server 'fibonacci' should be available";

  auto goal_msg = Fibonacci::Goal();
  goal_msg.order = 8;

  auto send_goal_future = client->async_send_goal(goal_msg);
  auto goal_status = executor.spin_until_future_complete(send_goal_future, 5s);
  ASSERT_EQ(goal_status, rclcpp::FutureReturnCode::SUCCESS);

  auto goal_handle = send_goal_future.get();
  ASSERT_NE(goal_handle, nullptr) << "Goal should be accepted";

  auto result_future = client->async_get_result(goal_handle);
  auto result_status = executor.spin_until_future_complete(result_future, 5s);
  ASSERT_EQ(result_status, rclcpp::FutureReturnCode::SUCCESS);

  auto wrapped_result = result_future.get();
  ASSERT_EQ(wrapped_result.code, rclcpp_action::ResultCode::SUCCEEDED)
    << "Goal should succeed";

  // Fibonacci(8): 0, 1, 1, 2, 3, 5, 8, 13
  auto & seq = wrapped_result.result->sequence;
  ASSERT_EQ(seq.size(), 8u);
  EXPECT_EQ(seq[0], 0);
  EXPECT_EQ(seq[1], 1);
  EXPECT_EQ(seq[7], 13);
}

TEST_F(ActionCompleteTest, ActionServerPublishesFeedback) {
  auto server_node = std::make_shared<FibonacciCompleteServer>();
  auto client_node = std::make_shared<rclcpp::Node>("test_feedback_client");
  auto client = rclcpp_action::create_client<Fibonacci>(client_node, "fibonacci");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server_node);
  executor.add_node(client_node);

  ASSERT_TRUE(client->wait_for_action_server(2s));

  auto goal_msg = Fibonacci::Goal();
  goal_msg.order = 6;

  std::mutex mtx;
  std::vector<size_t> feedback_lengths;

  auto send_goal_options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();
  send_goal_options.feedback_callback =
    [&mtx, &feedback_lengths](
      GoalHandleFibonacci::SharedPtr,
      const std::shared_ptr<const Fibonacci::Feedback> feedback) {
      std::lock_guard<std::mutex> lock(mtx);
      feedback_lengths.push_back(feedback->partial_sequence.size());
    };

  auto send_goal_future = client->async_send_goal(goal_msg, send_goal_options);
  auto goal_status = executor.spin_until_future_complete(send_goal_future, 5s);
  ASSERT_EQ(goal_status, rclcpp::FutureReturnCode::SUCCESS);

  auto goal_handle = send_goal_future.get();
  ASSERT_NE(goal_handle, nullptr);

  auto result_future = client->async_get_result(goal_handle);
  auto result_status = executor.spin_until_future_complete(result_future, 5s);
  ASSERT_EQ(result_status, rclcpp::FutureReturnCode::SUCCESS);

  // Feedback should have been published during execution
  std::lock_guard<std::mutex> lock(mtx);
  EXPECT_GT(feedback_lengths.size(), 0u)
    << "Student's server should publish feedback during execution";
}
