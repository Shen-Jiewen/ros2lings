#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <string>

class NodeLifecycleTest : public ::testing::Test {
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

// 声明外部变量
extern std::string answer_node_name;
extern std::string answer_default_namespace;
extern std::string answer_init_purpose;
extern std::string answer_spin_purpose;
extern std::string answer_default_topics;

TEST_F(NodeLifecycleTest, NodeNameAnswer) {
  EXPECT_EQ(answer_node_name, "my_node")
    << "Node(\"my_node\") 创建后，get_name() 返回 \"my_node\"";
}

TEST_F(NodeLifecycleTest, DefaultNamespaceAnswer) {
  EXPECT_EQ(answer_default_namespace, "/")
    << "默认命名空间是 \"/\"";

  // 实际验证
  auto node = std::make_shared<rclcpp::Node>("verify_ns_node");
  EXPECT_EQ(std::string(node->get_namespace()), "/");
}

TEST_F(NodeLifecycleTest, InitPurposeAnswer) {
  EXPECT_EQ(answer_init_purpose, "init_context")
    << "rclcpp::init() 的主要作用是初始化 ROS2 上下文（Context）";
}

TEST_F(NodeLifecycleTest, SpinPurposeAnswer) {
  EXPECT_EQ(answer_spin_purpose, "event_loop")
    << "rclcpp::spin() 启动事件循环，处理回调";
}

TEST_F(NodeLifecycleTest, DefaultTopicsAnswer) {
  EXPECT_EQ(answer_default_topics, "rosout")
    << "每个节点默认都有 /rosout 话题用于日志输出";
}
