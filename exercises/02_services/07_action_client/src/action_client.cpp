// I AM NOT DONE
//
// 练习: action_client
// 模块: 02 - Services & Actions
// 难度: ★★★☆☆
//
// 学习目标:
//   理解 ROS2 Action Client 的 SendGoalOptions 配置，
//   包括 feedback_callback 和 result_callback 的注册。
//
// 说明:
//   下面的代码尝试创建一个 Fibonacci Action Client，但代码中有三个错误。
//   你需要找到并修复它们。
//
// 步骤:
//   1. 添加 feedback_callback 的绑定
//   2. 添加 result_callback 的绑定
//   3. 修复 async_send_goal 的调用（需要传入 send_goal_options）
//   4. 修复完成后，删除文件顶部的 "// I AM NOT DONE"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <ros2lings_interfaces/action/fibonacci.hpp>
#include <memory>
#include <functional>

using Fibonacci = ros2lings_interfaces::action::Fibonacci;
using GoalHandleFibonacci = rclcpp_action::ClientGoalHandle<Fibonacci>;

class FibonacciActionClient : public rclcpp::Node
{
public:
  FibonacciActionClient() : Node("fibonacci_action_client")
  {
    client_ = rclcpp_action::create_client<Fibonacci>(this, "fibonacci");
    RCLCPP_INFO(get_logger(), "Fibonacci Action Client created");
  }

  void send_goal(int order)
  {
    if (!client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(get_logger(), "Action server not available");
      return;
    }

    auto goal_msg = Fibonacci::Goal();
    goal_msg.order = order;

    auto send_goal_options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();

    send_goal_options.goal_response_callback =
      std::bind(&FibonacciActionClient::goal_response_callback, this, std::placeholders::_1);

    // BUG 1: 缺少 feedback_callback 的绑定

    // BUG 2: 缺少 result_callback 的绑定

    // BUG 3: 没有传入 send_goal_options
    client_->async_send_goal(goal_msg);

    RCLCPP_INFO(get_logger(), "Goal sent");
  }

private:
  rclcpp_action::Client<Fibonacci>::SharedPtr client_;
  bool goal_accepted_ = false;
  bool result_received_ = false;

  void goal_response_callback(const GoalHandleFibonacci::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
    } else {
      goal_accepted_ = true;
      RCLCPP_INFO(get_logger(), "Goal accepted by server");
    }
  }

  void feedback_callback(
    GoalHandleFibonacci::SharedPtr,
    const std::shared_ptr<const Fibonacci::Feedback> feedback)
  {
    RCLCPP_INFO(get_logger(), "Received feedback: sequence length = %zu",
      feedback->partial_sequence.size());
  }

  void result_callback(const GoalHandleFibonacci::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        result_received_ = true;
        RCLCPP_INFO(get_logger(), "Result received: sequence length = %zu",
          result.result->sequence.size());
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(get_logger(), "Goal was aborted");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_INFO(get_logger(), "Goal was canceled");
        break;
      default:
        RCLCPP_ERROR(get_logger(), "Unknown result code");
        break;
    }
  }

public:
  bool is_goal_accepted() const { return goal_accepted_; }
  bool is_result_received() const { return result_received_; }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto client = std::make_shared<FibonacciActionClient>();
  client->send_goal(10);
  rclcpp::spin(client);
  rclcpp::shutdown();
  return 0;
}
