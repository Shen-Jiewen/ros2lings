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

    callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&DynamicParamNode::param_callback, this, std::placeholders::_1));
  }

  int get_speed() const { return speed_; }

private:
  rcl_interfaces::msg::SetParametersResult param_callback(
    const std::vector<rclcpp::Parameter> & params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto & param : params) {
      if (param.get_name() == "speed") {
        speed_ = param.as_int();
        RCLCPP_INFO(this->get_logger(), "speed changed to: %ld", param.as_int());
      }
    }

    return result;
  }

  int speed_ = 10;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_handle_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DynamicParamNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
