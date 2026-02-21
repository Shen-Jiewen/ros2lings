#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

// 在测试中声明正确答案来验证学习者的回答
class TopicIntrospectionTest : public ::testing::Test {
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

// 声明外部变量（来自 src/topic_introspection.cpp 的全局变量）
extern std::string answer_topic_list_purpose;
extern std::string answer_type_command;
extern std::string answer_string_field;
extern std::string answer_echo_command;
extern std::string answer_topic_exists_without_pub;

TEST_F(TopicIntrospectionTest, TopicListPurpose) {
  EXPECT_EQ(answer_topic_list_purpose, "list_topics")
    << "ros2 topic list 用于列出所有活跃话题";
}

TEST_F(TopicIntrospectionTest, TypeCommand) {
  EXPECT_EQ(answer_type_command, "ros2 topic type")
    << "ros2 topic type 用于查看话题的消息类型";
}

TEST_F(TopicIntrospectionTest, StringField) {
  EXPECT_EQ(answer_string_field, "data")
    << "std_msgs/msg/String 的字段名为 data";
}

TEST_F(TopicIntrospectionTest, EchoCommand) {
  EXPECT_EQ(answer_echo_command, "ros2 topic echo")
    << "ros2 topic echo 用于实时查看话题消息";
}

TEST_F(TopicIntrospectionTest, TopicExistsWithoutPublisher) {
  EXPECT_EQ(answer_topic_exists_without_pub, "no")
    << "没有活跃的发布者或订阅者时，话题不会出现在 topic list 中";
}
