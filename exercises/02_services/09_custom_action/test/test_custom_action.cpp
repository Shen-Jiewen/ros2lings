#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "ros2lings_24_custom_action/action/countdown.hpp"
#include <memory>
#include <chrono>

// Include student source directly so class definitions are visible in this translation unit
#include "../src/custom_action.cpp"

// Countdown and GoalHandleCountdown already defined in the included source
using namespace std::chrono_literals;
using ClientGoalHandle = rclcpp_action::ClientGoalHandle<Countdown>;

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

TEST_F(CustomActionTest, CanCreateServer) {
  auto server = std::make_shared<CountdownActionServer>();
  ASSERT_NE(server, nullptr);
  EXPECT_EQ(std::string(server->get_name()), "countdown_action_server");
}

TEST_F(CustomActionTest, GoalMessageHasTargetNumber) {
  auto goal = Countdown::Goal();
  goal.target_number = 10;
  EXPECT_EQ(goal.target_number, 10);
}

TEST_F(CustomActionTest, GoalRejectedForNonPositiveTarget) {
  auto server_node = std::make_shared<CountdownActionServer>();
  auto client_node = std::make_shared<rclcpp::Node>("test_countdown_reject_client");
  auto client = rclcpp_action::create_client<Countdown>(client_node, "countdown");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server_node);
  executor.add_node(client_node);

  ASSERT_TRUE(client->wait_for_action_server(2s))
    << "Student's 'countdown' action server should be available";

  auto goal_msg = Countdown::Goal();
  goal_msg.target_number = 0;

  auto send_future = client->async_send_goal(goal_msg);
  auto status = executor.spin_until_future_complete(send_future, 5s);
  ASSERT_EQ(status, rclcpp::FutureReturnCode::SUCCESS);

  auto goal_handle = send_future.get();
  EXPECT_EQ(goal_handle, nullptr)
    << "Goal with target_number=0 should be rejected";
}

TEST_F(CustomActionTest, CountdownSucceeds) {
  auto server_node = std::make_shared<CountdownActionServer>();
  auto client_node = std::make_shared<rclcpp::Node>("test_countdown_success_client");
  auto client = rclcpp_action::create_client<Countdown>(client_node, "countdown");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server_node);
  executor.add_node(client_node);

  ASSERT_TRUE(client->wait_for_action_server(2s));

  auto goal_msg = Countdown::Goal();
  goal_msg.target_number = 3;

  auto send_future = client->async_send_goal(goal_msg);
  auto goal_status = executor.spin_until_future_complete(send_future, 5s);
  ASSERT_EQ(goal_status, rclcpp::FutureReturnCode::SUCCESS);

  auto goal_handle = send_future.get();
  ASSERT_NE(goal_handle, nullptr) << "Goal should be accepted";

  auto result_future = client->async_get_result(goal_handle);
  auto result_status = executor.spin_until_future_complete(result_future, 10s);
  ASSERT_EQ(result_status, rclcpp::FutureReturnCode::SUCCESS);

  auto wrapped_result = result_future.get();
  ASSERT_EQ(wrapped_result.code, rclcpp_action::ResultCode::SUCCEEDED)
    << "Countdown should succeed";

  EXPECT_EQ(wrapped_result.result->final_count, 0)
    << "Final count should be 0 after countdown completes";
}
