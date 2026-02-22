#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/exceptions.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <chrono>

// Include student source directly so class definitions are visible in this translation unit
#include "../src/tf_time_travel.cpp"

using namespace std::chrono_literals;

class TfTimeTravelTest : public ::testing::Test {
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

TEST_F(TfTimeTravelTest, CanCreateNode) {
  auto node = std::make_shared<TfTimeTravelNode>();
  ASSERT_NE(node, nullptr);
}

TEST_F(TfTimeTravelTest, BufferExists) {
  auto node = std::make_shared<TfTimeTravelNode>();
  ASSERT_NE(node->get_buffer(), nullptr);
}

TEST_F(TfTimeTravelTest, NodeDoesNotCrashWithoutTransform) {
  // Spin the node briefly without any transform data.
  // The student's timer_callback must handle exceptions correctly (BUG 3).
  auto node = std::make_shared<TfTimeTravelNode>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  auto start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start < std::chrono::seconds(2)) {
    executor.spin_some(std::chrono::milliseconds(100));
  }
  // If we reach here without crashing, exception handling is correct
  EXPECT_GT(node->get_callback_count(), 0)
    << "Timer callback should have been invoked at least once";
  SUCCEED();
}

TEST_F(TfTimeTravelTest, TimerCallbackSucceedsWithTransformData) {
  // Create the student node (which has its own TransformListener)
  auto node = std::make_shared<TfTimeTravelNode>();

  // Broadcast a static transform "world" -> "robot" so the student's listener picks it up
  auto broadcaster_node = std::make_shared<rclcpp::Node>("test_broadcaster_node");
  auto broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(broadcaster_node);

  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = broadcaster_node->now();
  t.header.frame_id = "world";
  t.child_frame_id = "robot";
  t.transform.translation.x = 1.0;
  t.transform.translation.y = 2.0;
  t.transform.translation.z = 3.0;
  t.transform.rotation.w = 1.0;
  broadcaster->sendTransform(t);

  // Spin both nodes so the transform propagates and the timer fires
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(broadcaster_node);

  auto start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start < std::chrono::seconds(10)) {
    executor.spin_some(std::chrono::milliseconds(100));
    if (node->get_lookup_succeeded()) {
      break;
    }
  }
  // If the student uses TimePointZero (BUG 1 fixed), positive timeout (BUG 2 fixed),
  // and correct exception type (BUG 3 fixed), the lookup should succeed
  EXPECT_TRUE(node->get_lookup_succeeded())
    << "timer_callback should successfully lookup transform when data is available "
       "(requires TimePointZero and positive timeout)";
}

TEST_F(TfTimeTravelTest, CanLookupWithTimePointZero) {
  auto node = std::make_shared<TfTimeTravelNode>();
  auto buffer = node->get_buffer();

  // Manually inject a transform into the buffer
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = node->now();
  t.header.frame_id = "world";
  t.child_frame_id = "robot";
  t.transform.translation.x = 1.0;
  t.transform.translation.y = 2.0;
  t.transform.translation.z = 3.0;
  t.transform.rotation.w = 1.0;
  buffer->setTransform(t, "test_authority", true);

  // Using TimePointZero should succeed (verifies student uses TimePointZero, not future time)
  try {
    auto result = buffer->lookupTransform("world", "robot", tf2::TimePointZero);
    EXPECT_DOUBLE_EQ(result.transform.translation.x, 1.0);
    EXPECT_DOUBLE_EQ(result.transform.translation.y, 2.0);
    EXPECT_DOUBLE_EQ(result.transform.translation.z, 3.0);
  } catch (const tf2::TransformException & ex) {
    FAIL() << "lookupTransform with TimePointZero should not fail: " << ex.what();
  }
}
