# 提示 1

Action Client 发送目标时，需要通过 `SendGoalOptions` 注册所有回调。
检查 `send_goal_options` 中是否设置了 `feedback_callback` 和 `result_callback`。
另外，`async_send_goal` 需要两个参数。
