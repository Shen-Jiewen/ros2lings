# 提示 1

Action Server 的回调函数有严格的签名要求。
注意 `handle_goal` 应该返回什么类型？`bool` 还是某种枚举？
查看 `rclcpp_action::GoalResponse` 枚举。
