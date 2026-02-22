#include <rclcpp/rclcpp.hpp>

class ParameterNode : public rclcpp::Node
{
public:
  ParameterNode() : Node("parameter_node")
  {
    this->declare_parameter("max_speed", 10);
    this->declare_parameter("robot_name", "ros2bot");
    this->declare_parameter("update_frequency", 30.0);

    speed_ = this->get_parameter("max_speed").as_int();
    name_ = this->get_parameter("robot_name").as_string();
    freq_ = this->get_parameter("update_frequency").as_double();

    RCLCPP_INFO(this->get_logger(), "max_speed: %d", speed_);
    RCLCPP_INFO(this->get_logger(), "robot_name: %s", name_.c_str());
    RCLCPP_INFO(this->get_logger(), "update_frequency: %.1f", freq_);
  }

  int get_speed() const { return speed_; }
  std::string get_name_value() const { return name_; }
  double get_frequency() const { return freq_; }

private:
  int speed_ = 0;
  std::string name_;
  double freq_ = 0.0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ParameterNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
