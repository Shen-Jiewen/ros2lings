# 提示 2

倒数循环的结构:
```cpp
for (int count = goal->target_number; count >= 0; --count) {
  feedback->current_count = count;
  goal_handle->publish_feedback(feedback);
  // 可选: 检查取消
}
```

handle_goal 中的条件判断:
```cpp
if (goal->target_number <= 0) {
  return rclcpp_action::GoalResponse::REJECT;
}
return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
```
