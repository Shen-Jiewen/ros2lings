#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <ros2lings_interfaces/action/fibonacci.hpp>
#include <memory>
#include <thread>
#include <chrono>

// Include student source directly so class definitions are visible in this translation unit
#include "../src/action_client.cpp"

// Fibonacci and GoalHandleFibonacci already defined in the included source
using GoalHandleServer = rclcpp_action::ServerGoalHandle<Fibonacci>;
using GoalHandleClient = rclcpp_action::ClientGoalHandle<Fibonacci>;
using namespace std::chrono_literals;

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

TEST_F(ActionClientTest, StudentNodeCanBeCreated) {
  auto client = std::make_shared<FibonacciActionClient>();
  ASSERT_NE(client, nullptr);
  EXPECT_EQ(std::string(client->get_name()), "fibonacci_action_client");
}

TEST_F(ActionClientTest, StudentClientSendsGoalAndReceivesResult) {
  // Create a test action server
  auto server_node = std::make_shared<rclcpp::Node>("test_action_server");
  auto action_server = rclcpp_action::create_server<Fibonacci>(
    server_node,
    "fibonacci",
    // handle_goal
    [](const rclcpp_action::GoalUUID &, std::shared_ptr<const Fibonacci::Goal>) {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    // handle_cancel
    [](const std::shared_ptr<GoalHandleServer>) {
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    // handle_accepted
    [](const std::shared_ptr<GoalHandleServer> goal_handle) {
      std::thread([goal_handle]() {
        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<Fibonacci::Result>();
        result->sequence.push_back(0);
        result->sequence.push_back(1);
        for (int i = 2; i < goal->order; ++i) {
          result->sequence.push_back(
            result->sequence[i - 1] + result->sequence[i - 2]);
        }
        goal_handle->succeed(result);
      }).detach();
    });

  // Create the student's action client
  auto client_node = std::make_shared<FibonacciActionClient>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server_node);
  executor.add_node(client_node);

  // send_goal should return true since the server is available
  bool send_result = client_node->send_goal(5);
  ASSERT_TRUE(send_result) << "send_goal should return true when server is available";

  // Spin to process the goal and result
  auto start = std::chrono::steady_clock::now();
  while (!client_node->is_result_received() &&
         (std::chrono::steady_clock::now() - start) < 5s) {
    executor.spin_some(50ms);
  }

  EXPECT_TRUE(client_node->is_goal_accepted())
    << "Student's client should report goal accepted";
  EXPECT_TRUE(client_node->is_result_received())
    << "Student's client should report result received";
}
