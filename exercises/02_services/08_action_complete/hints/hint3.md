# 提示 3

完整的 `execute` 实现:

```cpp
for (int i = 2; i < goal->order; ++i) {
  if (goal_handle->is_canceling()) {
    result->sequence = partial_sequence;
    goal_handle->canceled(result);
    RCLCPP_INFO(get_logger(), "Goal canceled");
    return;
  }
  partial_sequence.push_back(partial_sequence[i - 1] + partial_sequence[i - 2]);
  goal_handle->publish_feedback(feedback);
}

result->sequence = partial_sequence;
goal_handle->succeed(result);
```
