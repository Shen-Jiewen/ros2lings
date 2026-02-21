#include <rclcpp/rclcpp.hpp>
#include <ros2lings_05_custom_message/msg/sensor_data.hpp>
#include <chrono>

using namespace std::chrono_literals;

class CustomMessageNode : public rclcpp::Node
{
public:
  CustomMessageNode() : Node("custom_message_node")
  {
    publisher_ = create_publisher<ros2lings_05_custom_message::msg::SensorData>("sensor_data", 10);

    subscription_ = create_subscription<ros2lings_05_custom_message::msg::SensorData>(
      "sensor_data", 10,
      std::bind(&CustomMessageNode::sub_callback, this, std::placeholders::_1));

    timer_ = create_wall_timer(200ms, std::bind(&CustomMessageNode::timer_callback, this));
  }

  std::string get_last_sensor_id() const { return last_sensor_id_; }
  double get_last_temperature() const { return last_temperature_; }

private:
  void timer_callback()
  {
    auto msg = ros2lings_05_custom_message::msg::SensorData();
    msg.temperature = 25.5;
    msg.humidity = 60.0;
    msg.sensor_id = "sensor_01";
    RCLCPP_INFO(get_logger(), "Publishing: temp=%.1f, humidity=%.1f, id=%s",
      msg.temperature, msg.humidity, msg.sensor_id.c_str());
    publisher_->publish(msg);
  }

  void sub_callback(const ros2lings_05_custom_message::msg::SensorData::SharedPtr msg)
  {
    RCLCPP_INFO(get_logger(), "Received: temp=%.1f, humidity=%.1f, id=%s",
      msg->temperature, msg->humidity, msg->sensor_id.c_str());
    last_sensor_id_ = msg->sensor_id;
    last_temperature_ = msg->temperature;
  }

  rclcpp::Publisher<ros2lings_05_custom_message::msg::SensorData>::SharedPtr publisher_;
  rclcpp::Subscription<ros2lings_05_custom_message::msg::SensorData>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string last_sensor_id_;
  double last_temperature_ = 0.0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CustomMessageNode>());
  rclcpp::shutdown();
  return 0;
}
