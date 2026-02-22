#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <ros2lings_interfaces/srv/add_two_ints.hpp>
#include <chrono>
#include <memory>

// Include student source directly so class definitions are visible in this translation unit
#include "../src/service_pair.cpp"

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

TEST_F(ServicePairTest, StudentServerNodeCanBeCreated) {
  auto server = std::make_shared<ServerNode>();
  ASSERT_NE(server, nullptr);
  EXPECT_EQ(std::string(server->get_name()), "server_node");
}

TEST_F(ServicePairTest, ServerAndClientCommunicate) {
  auto server_node = std::make_shared<ServerNode>();
  auto client_node = std::make_shared<rclcpp::Node>("test_client");
  auto client = client_node->create_client<AddTwoInts>("add_two_ints");

  ASSERT_TRUE(client->wait_for_service(2s))
    << "Student's service 'add_two_ints' should be available";

  auto request = std::make_shared<AddTwoInts::Request>();
  request->a = 42;
  request->b = 58;
  auto future = client->async_send_request(request);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server_node);
  executor.add_node(client_node);
  auto status = executor.spin_until_future_complete(future, 2s);
  ASSERT_EQ(status, rclcpp::FutureReturnCode::SUCCESS);

  auto response = future.get();
  EXPECT_EQ(response->sum, 100) << "42 + 58 should equal 100";
}

TEST_F(ServicePairTest, ServerAndClientMultipleCalls) {
  auto server_node = std::make_shared<ServerNode>();
  auto client_node = std::make_shared<rclcpp::Node>("test_multi_client");
  auto client = client_node->create_client<AddTwoInts>("add_two_ints");
  ASSERT_TRUE(client->wait_for_service(2s));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server_node);
  executor.add_node(client_node);

  // First call
  auto req1 = std::make_shared<AddTwoInts::Request>();
  req1->a = 10;
  req1->b = 20;
  auto fut1 = client->async_send_request(req1);
  ASSERT_EQ(executor.spin_until_future_complete(fut1, 2s),
            rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_EQ(fut1.get()->sum, 30);

  // Second call
  auto req2 = std::make_shared<AddTwoInts::Request>();
  req2->a = 0;
  req2->b = 0;
  auto fut2 = client->async_send_request(req2);
  ASSERT_EQ(executor.spin_until_future_complete(fut2, 2s),
            rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_EQ(fut2.get()->sum, 0) << "0 + 0 should equal 0";
}
