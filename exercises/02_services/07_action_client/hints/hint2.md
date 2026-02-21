# 提示 2

- 反馈回调绑定: `send_goal_options.feedback_callback = std::bind(&ClassName::feedback_callback, this, _1, _2)`
- 结果回调绑定: `send_goal_options.result_callback = std::bind(&ClassName::result_callback, this, _1)`
- 发送目标: `client_->async_send_goal(goal_msg, send_goal_options)` — 需要传入 options
