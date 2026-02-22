#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <ros2lings_interfaces/srv/add_two_ints.hpp>
#include <chrono>
#include <memory>

// Include student source directly so class definitions are visible in this translation unit
#include "../src/first_service.cpp"

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

TEST_F(FirstServiceTest, StudentNodeCanBeCreated) {
  // The student's AddTwoIntsServer class must be instantiable
  auto server = std::make_shared<AddTwoIntsServer>();
  ASSERT_NE(server, nullptr);
  EXPECT_EQ(std::string(server->get_name()), "add_two_ints_server");
}

TEST_F(FirstServiceTest, ServiceIsAvailable) {
  auto server = std::make_shared<AddTwoIntsServer>();
  auto client_node = std::make_shared<rclcpp::Node>("test_client_node");
  auto client = client_node->create_client<AddTwoInts>("add_two_ints");
  ASSERT_TRUE(client->wait_for_service(2s))
    << "Student's service 'add_two_ints' should be available";
}

TEST_F(FirstServiceTest, ServiceComputesCorrectSum) {
  auto server = std::make_shared<AddTwoIntsServer>();
  auto client_node = std::make_shared<rclcpp::Node>("test_client_compute");
  auto client = client_node->create_client<AddTwoInts>("add_two_ints");
  ASSERT_TRUE(client->wait_for_service(2s));

  auto request = std::make_shared<AddTwoInts::Request>();
  request->a = 3;
  request->b = 7;
  auto future = client->async_send_request(request);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server);
  executor.add_node(client_node);
  auto status = executor.spin_until_future_complete(future, 2s);
  ASSERT_EQ(status, rclcpp::FutureReturnCode::SUCCESS);

  auto response = future.get();
  EXPECT_EQ(response->sum, 10) << "3 + 7 should equal 10";
}

TEST_F(FirstServiceTest, ServiceHandlesNegativeNumbers) {
  auto server = std::make_shared<AddTwoIntsServer>();
  auto client_node = std::make_shared<rclcpp::Node>("test_client_neg");
  auto client = client_node->create_client<AddTwoInts>("add_two_ints");
  ASSERT_TRUE(client->wait_for_service(2s));

  auto request = std::make_shared<AddTwoInts::Request>();
  request->a = -5;
  request->b = 3;
  auto future = client->async_send_request(request);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server);
  executor.add_node(client_node);
  auto status = executor.spin_until_future_complete(future, 2s);
  ASSERT_EQ(status, rclcpp::FutureReturnCode::SUCCESS);

  auto response = future.get();
  EXPECT_EQ(response->sum, -2) << "-5 + 3 should equal -2";
}
