#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <ros2lings_interfaces/srv/add_two_ints.hpp>
#include <chrono>
#include <memory>

using namespace std::chrono_literals;
using AddTwoInts = ros2lings_interfaces::srv::AddTwoInts;

class FirstServiceTest : public ::testing::Test {
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

TEST_F(FirstServiceTest, CanCreateServiceServer) {
  auto node = std::make_shared<rclcpp::Node>("test_srv_node");
  auto service = node->create_service<AddTwoInts>(
    "add_two_ints",
    [](const std::shared_ptr<AddTwoInts::Request> request,
       std::shared_ptr<AddTwoInts::Response> response) {
      response->sum = request->a + request->b;
    });
  ASSERT_NE(service, nullptr);
}

TEST_F(FirstServiceTest, ServiceComputesCorrectSum) {
  auto node = std::make_shared<rclcpp::Node>("test_srv_compute");

  // 创建服务服务器
  auto service = node->create_service<AddTwoInts>(
    "add_two_ints",
    [](const std::shared_ptr<AddTwoInts::Request> request,
       std::shared_ptr<AddTwoInts::Response> response) {
      response->sum = request->a + request->b;
    });

  // 创建客户端
  auto client = node->create_client<AddTwoInts>("add_two_ints");

  // 等待服务可用
  ASSERT_TRUE(client->wait_for_service(2s)) << "服务应当在超时前可用";

  // 发送请求
  auto request = std::make_shared<AddTwoInts::Request>();
  request->a = 3;
  request->b = 7;
  auto future = client->async_send_request(request);

  // 等待结果
  auto status = rclcpp::spin_until_future_complete(node, future, 2s);
  ASSERT_EQ(status, rclcpp::FutureReturnCode::SUCCESS) << "服务调用应当成功";

  auto response = future.get();
  EXPECT_EQ(response->sum, 10) << "3 + 7 应当等于 10";
}

TEST_F(FirstServiceTest, ServiceHandlesNegativeNumbers) {
  auto node = std::make_shared<rclcpp::Node>("test_srv_negative");

  auto service = node->create_service<AddTwoInts>(
    "add_two_ints",
    [](const std::shared_ptr<AddTwoInts::Request> request,
       std::shared_ptr<AddTwoInts::Response> response) {
      response->sum = request->a + request->b;
    });

  auto client = node->create_client<AddTwoInts>("add_two_ints");
  ASSERT_TRUE(client->wait_for_service(2s));

  auto request = std::make_shared<AddTwoInts::Request>();
  request->a = -5;
  request->b = 3;
  auto future = client->async_send_request(request);

  auto status = rclcpp::spin_until_future_complete(node, future, 2s);
  ASSERT_EQ(status, rclcpp::FutureReturnCode::SUCCESS);

  auto response = future.get();
  EXPECT_EQ(response->sum, -2) << "-5 + 3 应当等于 -2";
}
