// I AM NOT DONE
//
// 练习: multi_node_process
// 模块: 01 - Nodes & Topics
// 难度: ★★☆☆☆
//
// 学习目标:
//   在单个进程中运行多个节点，使用 Executor 管理它们。
//
// 说明:
//   下面提供了两个节点类：ProducerNode（发布者）和 ConsumerNode（订阅者）。
//   你需要在 main 函数中设置一个 MultiThreadedExecutor，
//   将两个节点都添加进去，然后运行 executor。
//
// 背景知识:
//   - SingleThreadedExecutor: 单线程依次处理所有节点的回调
//   - MultiThreadedExecutor: 多线程并行处理回调
//   - 一个进程中可以运行多个节点，它们共享同一个 ROS2 上下文
//
// 步骤:
//   1. 创建 ProducerNode 和 ConsumerNode 的实例
//   2. 创建 MultiThreadedExecutor
//   3. 将两个节点添加到 executor
//   4. 运行 executor
//   5. 修复完成后，删除文件顶部的 "// I AM NOT DONE"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

// 生产者节点：每 200ms 发布一条消息
class ProducerNode : public rclcpp::Node
{
public:
  ProducerNode() : Node("producer_node")
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("internal_topic", 10);
    timer_ = this->create_wall_timer(200ms, [this]() {
      auto msg = std_msgs::msg::String();
      msg.data = "生产消息 #" + std::to_string(count_++);
      RCLCPP_INFO(this->get_logger(), "发布: '%s'", msg.data.c_str());
      publisher_->publish(msg);
    });
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_ = 0;
};

// 消费者节点：订阅并处理消息
class ConsumerNode : public rclcpp::Node
{
public:
  ConsumerNode() : Node("consumer_node"), received_count_(0)
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "internal_topic", 10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "收到: '%s'", msg->data.c_str());
        received_count_++;
      });
  }

  int get_received_count() const { return received_count_; }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  int received_count_;
};

#ifndef ROS2LINGS_TEST
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // TODO: 创建 ProducerNode 实例（使用 std::make_shared）
  // auto producer = ???;

  // TODO: 创建 ConsumerNode 实例（使用 std::make_shared）
  // auto consumer = ???;

  // TODO: 创建 rclcpp::executors::MultiThreadedExecutor
  // auto executor = ???;

  // TODO: 使用 executor.add_node() 将两个节点添加到 executor
  // ???

  // TODO: 运行 executor（提示：executor.spin()）
  // ???

  rclcpp::shutdown();
  return 0;
}
#endif
