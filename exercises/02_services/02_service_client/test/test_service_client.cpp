#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <ros2lings_interfaces/srv/add_two_ints.hpp>
#include <chrono>
#include <memory>

// Include student source directly so class definitions are visible in this translation unit
#include "../src/service_client.cpp"

using namespace std::chrono_literals;
using AddTwoInts = ros2lings_interfaces::srv::AddTwoInts;

class ServiceClientTest : public ::testing::Test {
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

TEST_F(ServiceClientTest, StudentSourceCompiles) {
  // The student source (service_client.cpp) is compiled alongside this test.
  // If the student's code has compile errors (e.g. passing a raw pointer to
  // async_send_request), the build will fail before this test runs.
  SUCCEED();
}

TEST_F(ServiceClientTest, ClientCanWaitForServiceAndCall) {
  // Verify the complete client workflow the student must implement
  auto server_node = std::make_shared<rclcpp::Node>("test_server");
  auto service = server_node->create_service<AddTwoInts>(
    "add_two_ints",
    [](const std::shared_ptr<AddTwoInts::Request> request,
       std::shared_ptr<AddTwoInts::Response> response) {
      response->sum = request->a + request->b;
    });

  auto client_node = std::make_shared<rclcpp::Node>("test_client");
  auto client = client_node->create_client<AddTwoInts>("add_two_ints");

  // Pattern 1: wait_for_service before sending
  ASSERT_TRUE(client->wait_for_service(2s)) << "Service should be available";

  // Pattern 2: async_send_request with shared_ptr
  auto request = std::make_shared<AddTwoInts::Request>();
  request->a = 5;
  request->b = 3;
  auto future = client->async_send_request(request);

  // Pattern 3: spin_until_future_complete before getting result
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server_node);
  executor.add_node(client_node);
  auto status = executor.spin_until_future_complete(future, 2s);
  ASSERT_EQ(status, rclcpp::FutureReturnCode::SUCCESS);

  auto response = future.get();
  EXPECT_EQ(response->sum, 8) << "5 + 3 should equal 8";
}

TEST_F(ServiceClientTest, ClientHandlesLargeNumbers) {
  auto server_node = std::make_shared<rclcpp::Node>("test_large_server");
  auto service = server_node->create_service<AddTwoInts>(
    "add_two_ints",
    [](const std::shared_ptr<AddTwoInts::Request> request,
       std::shared_ptr<AddTwoInts::Response> response) {
      response->sum = request->a + request->b;
    });

  auto client_node = std::make_shared<rclcpp::Node>("test_large_client");
  auto client = client_node->create_client<AddTwoInts>("add_two_ints");
  ASSERT_TRUE(client->wait_for_service(2s));

  auto request = std::make_shared<AddTwoInts::Request>();
  request->a = 1000000;
  request->b = 2000000;
  auto future = client->async_send_request(request);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server_node);
  executor.add_node(client_node);
  auto status = executor.spin_until_future_complete(future, 2s);
  ASSERT_EQ(status, rclcpp::FutureReturnCode::SUCCESS);

  auto response = future.get();
  EXPECT_EQ(response->sum, 3000000);
}
