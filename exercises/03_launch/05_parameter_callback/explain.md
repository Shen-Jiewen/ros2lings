# 参数动态回调 — 运行时热更新参数

## 概念

ROS2 节点的参数可以在运行时被外部修改（通过 `ros2 param set` 命令或服务调用）。
为了响应这种修改，节点需要注册一个参数回调函数。
当参数被修改时，回调函数会被自动调用，节点可以在其中更新内部状态。

## 注册回调

```cpp
#include <rcl_interfaces/msg/set_parameters_result.hpp>

class MyNode : public rclcpp::Node {
public:
  MyNode() : Node("my_node") {
    this->declare_parameter("speed", 10);

    // 注册回调
    callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&MyNode::param_callback, this, std::placeholders::_1));
  }

private:
  rcl_interfaces::msg::SetParametersResult param_callback(
    const std::vector<rclcpp::Parameter> & params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto & param : params) {
      if (param.get_name() == "speed") {
        speed_ = param.as_int();
      }
    }
    return result;
  }

  int speed_ = 10;
  // 必须保存回调句柄！
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_handle_;
};
```

## 关键点

- `add_on_set_parameters_callback` 返回一个句柄，**必须保存**，
  否则回调会被立即销毁，不会触发
- 回调函数必须返回 `rcl_interfaces::msg::SetParametersResult`
- `result.successful = true` 表示接受参数修改；
  设为 `false` 会拒绝修改，参数值不变
- 回调接收的是 `std::vector<rclcpp::Parameter>`，可能包含多个参数
- 在回调中更新内部成员变量，节点才能使用新的参数值
- 外部修改参数：`ros2 param set /node_name speed 20`
