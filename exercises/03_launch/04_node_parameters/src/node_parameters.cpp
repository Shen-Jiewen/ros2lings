// I AM NOT DONE
//
// 练习: node_parameters
// 模块: 03 - Launch & Parameters
// 难度: ★★☆☆☆
//
// 学习目标:
//   理解 ROS2 节点中参数的声明（declare_parameter）和获取（get_parameter），
//   掌握参数类型必须匹配的规则。
//
// 说明:
//   下面的节点试图声明和读取三个参数，但有 3 个错误需要你修复。
//
// 步骤:
//   1. 修复 max_speed 的默认值 — 应为整数类型（如 10），而非字符串
//   2. 修复 get_parameter 中的参数名拼写错误
//   3. 取消注释 update_frequency 参数的声明和获取
//   4. 修复完成后，删除文件顶部的 "// I AM NOT DONE"

#include <rclcpp/rclcpp.hpp>

class ParameterNode : public rclcpp::Node
{
public:
  ParameterNode() : Node("parameter_node")
  {
    // BUG 1: 类型不匹配 — max_speed 应该是整数，但这里声明为字符串
    this->declare_parameter("max_speed", "fast");

    // 正确声明 robot_name 参数
    this->declare_parameter("robot_name", "ros2bot");

    // BUG 3: 缺少 update_frequency 参数的声明
    // this->declare_parameter("update_frequency", 30.0);

    // 获取参数值
    speed_ = this->get_parameter("max_speed").as_int();

    // BUG 2: 参数名拼写错误 — "robot_nam" 少了一个 'e'
    name_ = this->get_parameter("robot_nam").as_string();

    // BUG 3（续）: 缺少 update_frequency 的获取
    // freq_ = this->get_parameter("update_frequency").as_double();

    RCLCPP_INFO(this->get_logger(), "max_speed: %d", speed_);
    RCLCPP_INFO(this->get_logger(), "robot_name: %s", name_.c_str());
    // RCLCPP_INFO(this->get_logger(), "update_frequency: %.1f", freq_);
  }

  int get_speed() const { return speed_; }
  std::string get_name_value() const { return name_; }
  double get_frequency() const { return freq_; }

private:
  int speed_ = 0;
  std::string name_;
  double freq_ = 0.0;
};

#ifndef ROS2LINGS_TEST
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ParameterNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
#endif
