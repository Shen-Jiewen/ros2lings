// I AM NOT DONE
//
// 练习: custom_message
// 模块: 01 - Nodes & Topics
// 难度: ★★☆☆☆
//
// 学习目标:
//   学习如何定义自定义消息类型（.msg 文件）并在代码中使用。
//
// 说明:
//   本练习已经在 msg/SensorData.msg 中定义了一个自定义消息类型。
//   你需要完成节点代码，使其能发布和订阅这个自定义消息。
//
// 步骤:
//   1. 包含自动生成的自定义消息头文件
//   2. 创建 Publisher 和 Subscription 使用自定义消息类型
//   3. 在定时回调中填充消息字段并发布
//   4. 在订阅回调中读取消息字段
//   5. 修复完成后，删除文件顶部的 "// I AM NOT DONE"

#include <rclcpp/rclcpp.hpp>
#include <chrono>

// TODO: 包含自定义消息的头文件
// 提示: 包名转小写，消息名转蛇形命名
// #include <ros2lings_05_custom_message/msg/sensor_data.hpp>

using namespace std::chrono_literals;

class CustomMessageNode : public rclcpp::Node
{
public:
  CustomMessageNode() : Node("custom_message_node")
  {
    // TODO: 创建 Publisher，消息类型为自定义的 SensorData
    // 话题名 "sensor_data"，QoS 深度 10
    // publisher_ = ???

    // TODO: 创建 Subscription，订阅话题 "sensor_data"
    // subscription_ = ???

    timer_ = create_wall_timer(200ms, std::bind(&CustomMessageNode::timer_callback, this));
  }

  std::string get_last_sensor_id() const { return last_sensor_id_; }
  double get_last_temperature() const { return last_temperature_; }

private:
  void timer_callback()
  {
    // TODO: 创建 SensorData 消息并填充字段
    // auto msg = ros2lings_05_custom_message::msg::SensorData();
    // msg.temperature = 25.5;
    // msg.humidity = 60.0;
    // msg.sensor_id = "sensor_01";
    // publisher_->publish(msg);
  }

  // TODO: 实现订阅回调
  // void sub_callback(const ros2lings_05_custom_message::msg::SensorData::SharedPtr msg)
  // {
  //   last_sensor_id_ = msg->sensor_id;
  //   last_temperature_ = msg->temperature;
  // }

  // rclcpp::Publisher<ros2lings_05_custom_message::msg::SensorData>::SharedPtr publisher_;
  // rclcpp::Subscription<ros2lings_05_custom_message::msg::SensorData>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string last_sensor_id_;
  double last_temperature_ = 0.0;
};

#ifndef ROS2LINGS_TEST
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CustomMessageNode>());
  rclcpp::shutdown();
  return 0;
}
#endif
