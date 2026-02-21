#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <ros2lings_interfaces/srv/add_two_ints.hpp>
#include <chrono>
#include <memory>

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

TEST_F(ServiceClientTest, CanCreateClient) {
  auto node = std::make_shared<rclcpp::Node>("test_client_node");
  auto client = node->create_client<AddTwoInts>("add_two_ints");
  ASSERT_NE(client, nullptr);
}

TEST_F(ServiceClientTest, ClientCanWaitForService) {
  auto node = std::make_shared<rclcpp::Node>("test_wait_node");

  // 先创建服务器
  auto service = node->create_service<AddTwoInts>(
    "test_wait_srv",
    [](const std::shared_ptr<AddTwoInts::Request> request,
       std::shared_ptr<AddTwoInts::Response> response) {
      response->sum = request->a + request->b;
    });

  // 创建客户端
  auto client = node->create_client<AddTwoInts>("test_wait_srv");

  // wait_for_service 应该成功
  EXPECT_TRUE(client->wait_for_service(2s)) << "客户端应当能发现服务";
}

TEST_F(ServiceClientTest, AsyncCallReturnsCorrectResult) {
  auto node = std::make_shared<rclcpp::Node>("test_async_node");

  // 创建服务器
  auto service = node->create_service<AddTwoInts>(
    "test_async_srv",
    [](const std::shared_ptr<AddTwoInts::Request> request,
       std::shared_ptr<AddTwoInts::Response> response) {
      response->sum = request->a + request->b;
    });

  // 创建客户端
  auto client = node->create_client<AddTwoInts>("test_async_srv");
  ASSERT_TRUE(client->wait_for_service(2s));

  // 发送请求
  auto request = std::make_shared<AddTwoInts::Request>();
  request->a = 5;
  request->b = 3;
  auto future = client->async_send_request(request);

  // 使用 spin_until_future_complete 等待结果
  auto status = rclcpp::spin_until_future_complete(node, future, 2s);
  ASSERT_EQ(status, rclcpp::FutureReturnCode::SUCCESS) << "异步调用应当成功";

  auto response = future.get();
  EXPECT_EQ(response->sum, 8) << "5 + 3 应当等于 8";
}

TEST_F(ServiceClientTest, ClientHandlesLargeNumbers) {
  auto node = std::make_shared<rclcpp::Node>("test_large_node");

  auto service = node->create_service<AddTwoInts>(
    "test_large_srv",
    [](const std::shared_ptr<AddTwoInts::Request> request,
       std::shared_ptr<AddTwoInts::Response> response) {
      response->sum = request->a + request->b;
    });

  auto client = node->create_client<AddTwoInts>("test_large_srv");
  ASSERT_TRUE(client->wait_for_service(2s));

  auto request = std::make_shared<AddTwoInts::Request>();
  request->a = 1000000;
  request->b = 2000000;
  auto future = client->async_send_request(request);

  auto status = rclcpp::spin_until_future_complete(node, future, 2s);
  ASSERT_EQ(status, rclcpp::FutureReturnCode::SUCCESS);

  auto response = future.get();
  EXPECT_EQ(response->sum, 3000000) << "大数加法应当正确";
}
