// 参考答案 — first_action_server
// Fibonacci Action Server: 接收目标、计算序列、返回结果

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <ros2lings_interfaces/action/fibonacci.hpp>
#include <memory>
#include <thread>

using Fibonacci = ros2lings_interfaces::action::Fibonacci;
using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;

class FibonacciActionServer : public rclcpp::Node
{
public:
  FibonacciActionServer() : Node("fibonacci_action_server")
  {
    using namespace std::placeholders;

    action_server_ = rclcpp_action::create_server<Fibonacci>(
      this,
      "fibonacci",
      std::bind(&FibonacciActionServer::handle_goal, this, _1, _2),
      std::bind(&FibonacciActionServer::handle_cancel, this, _1),
      std::bind(&FibonacciActionServer::handle_accepted, this, _1));

    RCLCPP_INFO(get_logger(), "Fibonacci Action Server started");
  }

private:
  rclcpp_action::Server<Fibonacci>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Fibonacci::Goal> goal)
  {
    RCLCPP_INFO(get_logger(), "Received goal request with order %d", goal->order);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    RCLCPP_INFO(get_logger(), "Received cancel request");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    RCLCPP_INFO(get_logger(), "Goal accepted");
    std::thread{std::bind(&FibonacciActionServer::execute, this, goal_handle)}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    RCLCPP_INFO(get_logger(), "Executing goal");
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<Fibonacci::Result>();

    auto & sequence = result->sequence;
    sequence.push_back(0);
    sequence.push_back(1);
    for (int i = 2; i < goal->order; ++i) {
      sequence.push_back(sequence[i - 1] + sequence[i - 2]);
    }

    goal_handle->succeed(result);
    RCLCPP_INFO(get_logger(), "Goal finished");
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FibonacciActionServer>());
  rclcpp::shutdown();
  return 0;
}
