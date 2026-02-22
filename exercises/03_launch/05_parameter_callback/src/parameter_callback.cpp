// I AM NOT DONE
//
// 练习: parameter_callback
// 模块: 03 - Launch & Parameters
// 难度: ★★☆☆☆
//
// 学习目标:
//   学会使用 add_on_set_parameters_callback 来响应参数的动态修改，
//   实现运行时参数热更新。
//
// 说明:
//   下面的节点声明了一个 speed 参数，并希望在参数被修改时
//   自动更新内部状态。但有 3 个错误需要你修复。
//
// 步骤:
//   1. 注册参数回调 — 使用 add_on_set_parameters_callback
//   2. 回调函数必须返回 rcl_interfaces::msg::SetParametersResult
//   3. 回调中更新 speed_ 成员变量
//   4. 修复完成后，删除文件顶部的 "// I AM NOT DONE"

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <vector>

class DynamicParamNode : public rclcpp::Node
{
public:
  DynamicParamNode() : Node("dynamic_param_node")
  {
    this->declare_parameter("speed", 10);
    speed_ = this->get_parameter("speed").as_int();

    // BUG 1: 回调没有注册！
    // 提示: 使用 this->add_on_set_parameters_callback(...)
    //       需要用 std::bind 绑定 param_callback 方法
    // callback_handle_ = this->add_on_set_parameters_callback(
    //   std::bind(&DynamicParamNode::param_callback, this, std::placeholders::_1));
  }

  int get_speed() const { return speed_; }

private:
  rcl_interfaces::msg::SetParametersResult param_callback(
    const std::vector<rclcpp::Parameter> & params)
  {
    // BUG 2: 没有创建 result 对象，也没有返回正确的类型
    // 提示: 需要创建 SetParametersResult 并设置 successful = true

    for (const auto & param : params) {
      if (param.get_name() == "speed") {
        // BUG 3: 没有更新内部状态 speed_
        // 提示: speed_ = param.as_int();
        RCLCPP_INFO(this->get_logger(), "speed changed to: %ld", param.as_int());
      }
    }

    // BUG 2（续）: 这里应该返回 result，但目前什么都没返回
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = false;  // 应改为 true
    return result;
  }

  int speed_ = 10;
  // 需要保存回调句柄，否则回调会被销毁
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_handle_;
};

#ifndef ROS2LINGS_TEST
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DynamicParamNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
#endif
