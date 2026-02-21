# 提示 3

完整的修复：

1. 添加反馈回调:
   ```cpp
   send_goal_options.feedback_callback =
     std::bind(&FibonacciActionClient::feedback_callback, this,
       std::placeholders::_1, std::placeholders::_2);
   ```
2. 添加结果回调:
   ```cpp
   send_goal_options.result_callback =
     std::bind(&FibonacciActionClient::result_callback, this,
       std::placeholders::_1);
   ```
3. 修复发送目标:
   ```cpp
   client_->async_send_goal(goal_msg, send_goal_options);
   ```
