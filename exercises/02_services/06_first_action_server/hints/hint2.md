# 提示 2

- `handle_goal` 应返回 `rclcpp_action::GoalResponse`，而不是 `bool`
- `handle_cancel` 的参数应该是 `const std::shared_ptr<GoalHandleFibonacci>`，而不是原始指针
- `handle_accepted` 中应该启动线程: `std::thread{...}.detach()`
- 执行完成后需要调用 `goal_handle->succeed()` 设置结果
