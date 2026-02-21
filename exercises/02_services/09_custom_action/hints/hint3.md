# 提示 3

完整的 execute 实现:

```cpp
void execute(const std::shared_ptr<GoalHandleCountdown> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<Countdown::Feedback>();
  auto result = std::make_shared<Countdown::Result>();

  for (int count = goal->target_number; count >= 0; --count) {
    if (goal_handle->is_canceling()) {
      result->final_count = count;
      goal_handle->canceled(result);
      return;
    }
    feedback->current_count = count;
    goal_handle->publish_feedback(feedback);
  }

  result->final_count = 0;
  goal_handle->succeed(result);
}
```
