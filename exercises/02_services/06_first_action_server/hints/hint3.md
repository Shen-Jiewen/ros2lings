# 提示 3

完整的修复：

1. `handle_goal` 返回类型改为 `rclcpp_action::GoalResponse`，返回 `rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE`
2. `handle_cancel` 参数改为 `const std::shared_ptr<GoalHandleFibonacci> goal_handle`
3. `handle_accepted` 中添加:
   ```cpp
   std::thread{std::bind(&FibonacciActionServer::execute, this, goal_handle)}.detach();
   ```
4. `execute` 末尾添加:
   ```cpp
   goal_handle->succeed(result);
   ```
