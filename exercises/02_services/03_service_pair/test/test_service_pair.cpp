#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <ros2lings_interfaces/srv/add_two_ints.hpp>
#include <chrono>
#include <memory>

using namespace std::chrono_literals;
using AddTwoInts = ros2lings_interfaces::srv::AddTwoInts;

class ServicePairTest : public ::testing::Test {
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

TEST_F(ServicePairTest, ServerAndClientCommunicate) {
  // 创建服务器节点
  auto server_node = std::make_shared<rclcpp::Node>("test_server");
  auto service = server_node->create_service<AddTwoInts>(
    "add_two_ints",
    [](const std::shared_ptr<AddTwoInts::Request> request,
       std::shared_ptr<AddTwoInts::Response> response) {
      response->sum = request->a + request->b;
    });

  // 创建客户端节点
  auto client_node = std::make_shared<rclcpp::Node>("test_client");
  auto client = client_node->create_client<AddTwoInts>("add_two_ints");

  ASSERT_TRUE(client->wait_for_service(2s)) << "服务应当可用";

  // 发送请求
  auto request = std::make_shared<AddTwoInts::Request>();
  request->a = 42;
  request->b = 58;
  auto future = client->async_send_request(request);

  // 使用 executor 同时 spin 两个节点
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server_node);
  executor.add_node(client_node);
  auto status = executor.spin_until_future_complete(future, 2s);
  ASSERT_EQ(status, rclcpp::FutureReturnCode::SUCCESS) << "服务调用应当成功";

  auto response = future.get();
  EXPECT_EQ(response->sum, 100) << "42 + 58 应当等于 100";
}

TEST_F(ServicePairTest, ServerAndClientMultipleCalls) {
  auto server_node = std::make_shared<rclcpp::Node>("test_multi_server");
  auto service = server_node->create_service<AddTwoInts>(
    "add_multi",
    [](const std::shared_ptr<AddTwoInts::Request> request,
       std::shared_ptr<AddTwoInts::Response> response) {
      response->sum = request->a + request->b;
    });

  auto client_node = std::make_shared<rclcpp::Node>("test_multi_client");
  auto client = client_node->create_client<AddTwoInts>("add_multi");
  ASSERT_TRUE(client->wait_for_service(2s));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server_node);
  executor.add_node(client_node);

  // 第一次调用
  auto req1 = std::make_shared<AddTwoInts::Request>();
  req1->a = 10;
  req1->b = 20;
  auto fut1 = client->async_send_request(req1);
  ASSERT_EQ(executor.spin_until_future_complete(fut1, 2s),
            rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_EQ(fut1.get()->sum, 30);

  // 第二次调用
  auto req2 = std::make_shared<AddTwoInts::Request>();
  req2->a = 0;
  req2->b = 0;
  auto fut2 = client->async_send_request(req2);
  ASSERT_EQ(executor.spin_until_future_complete(fut2, 2s),
            rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_EQ(fut2.get()->sum, 0) << "0 + 0 应当等于 0";
}

TEST_F(ServicePairTest, WaitForServiceDetectsServer) {
  auto server_node = std::make_shared<rclcpp::Node>("test_detect_server");
  auto service = server_node->create_service<AddTwoInts>(
    "detect_srv",
    [](const std::shared_ptr<AddTwoInts::Request>,
       std::shared_ptr<AddTwoInts::Response>) {});

  auto client_node = std::make_shared<rclcpp::Node>("test_detect_client");
  auto client = client_node->create_client<AddTwoInts>("detect_srv");

  // 服务已存在，wait_for_service 应该立即返回 true
  EXPECT_TRUE(client->wait_for_service(1s));
}
