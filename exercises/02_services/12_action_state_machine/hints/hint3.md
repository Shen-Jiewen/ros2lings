# 提示 3

完整修复:

1. handle_cancel:
```cpp
return rclcpp_action::CancelResponse::ACCEPT;
```

2. handle_accepted:
```cpp
void handle_accepted(const std::shared_ptr<GoalHandleFibonacci> goal_handle) {
  std::thread{std::bind(&FibonacciStateMachine::execute, this, goal_handle)}.detach();
}
```

3. execute — 将 succeed 移到循环之后:
```cpp
void execute(const std::shared_ptr<GoalHandleFibonacci> goal_handle) {
  // ... 初始化 ...
  for (int i = 2; i < goal->order; ++i) {
    // 检查取消、计算、发布反馈
  }
  // 循环结束后才设置结果
  result->sequence = partial_sequence;
  goal_handle->succeed(result);
}
```
