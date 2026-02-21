# 提示 2

四个需要修复的地方:
1. `ActionServer(self, Fibonacci, 'fibonacci', self.execute_callback)` — 类型在前，名称在后
2. Fibonacci 用加法: `sequence[i-1] + sequence[i-2]`，不是乘法
3. 反馈方法是 `goal_handle.publish_feedback(feedback_msg)`，不是 `send_feedback`
4. `execute_callback` 必须 `return result`
