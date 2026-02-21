// I AM NOT DONE
//
// 练习: component_node
// 模块: 01 - Nodes & Topics
// 难度: ★★★☆☆
//
// 学习目标:
//   创建一个可组合（composable）节点，理解组件化节点的注册机制。
//
// 说明:
//   组件化节点允许多个节点在同一个进程中运行，共享内存以提高效率。
//   你需要完成以下工作：
//   1. 让类继承自 rclcpp::Node
//   2. 实现构造函数（接受 rclcpp::NodeOptions 参数）
//   3. 创建发布者和定时器
//   4. 注册组件（使用 RCLCPP_COMPONENTS_REGISTER_NODE 宏）
//
// 步骤:
//   1. 修复类的继承关系
//   2. 修复构造函数签名，使其接受 NodeOptions
//   3. 实现定时器回调中的发布逻辑
//   4. 在文件末尾注册组件
//   5. 修复完成后，删除文件顶部的 "// I AM NOT DONE"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>

using namespace std::chrono_literals;

// TODO: 这个类需要继承 rclcpp::Node
class TalkerComponent
{
public:
  // TODO: 组件节点的构造函数必须接受 const rclcpp::NodeOptions & 参数
  // 并且需要将节点名和 options 传递给基类
  TalkerComponent()
  {
    // TODO: 创建一个 std_msgs::msg::String 类型的发布者
    // 话题名: "component_chatter"，队列大小: 10
    publisher_ = ???;

    // TODO: 创建一个 500ms 的定时器，在回调中发布消息
    timer_ = ???;
  }

private:
  void timer_callback()
  {
    auto msg = std_msgs::msg::String();
    msg.data = "来自组件的消息 #" + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "发布: '%s'", msg.data.c_str());
    publisher_->publish(msg);
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_ = 0;
};

// TODO: 使用宏注册组件，使其可以被动态加载
// 提示: RCLCPP_COMPONENTS_REGISTER_NODE(???)
