# Action Server — 创建长时间运行的服务

## 概念

Action 是 ROS2 中用于长时间运行任务的通信机制。与 Service（同步请求-响应）不同，
Action 提供了目标(Goal)、反馈(Feedback)和结果(Result)三个阶段。

Action Server 负责接收目标、执行任务、发送反馈和返回结果。

## 回调结构

```
Client 发送 Goal
        |
        v
handle_goal()          ← 决定是否接受目标
        |
        v (ACCEPT)
handle_accepted()      ← 目标被接受，启动执行
        |
        v
execute()              ← 在新线程中执行任务
        |
        v
goal_handle->succeed() ← 设置结果并标记成功
```

## 三个回调的签名

```cpp
// 1. 处理目标请求 — 返回 GoalResponse
rclcpp_action::GoalResponse handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const Fibonacci::Goal> goal);

// 2. 处理取消请求 — 返回 CancelResponse
rclcpp_action::CancelResponse handle_cancel(
  const std::shared_ptr<GoalHandleFibonacci> goal_handle);

// 3. 目标被接受 — 启动执行线程
void handle_accepted(
  const std::shared_ptr<GoalHandleFibonacci> goal_handle);
```

## 关键点

- `handle_goal` 返回 `GoalResponse::ACCEPT_AND_EXECUTE` 或 `REJECT`
- `handle_cancel` 参数是 `shared_ptr`，不是原始指针
- `handle_accepted` 必须启动新线程执行，避免阻塞
- 执行完成后调用 `goal_handle->succeed(result)` 返回结果
- Action 基于 Service + Topic 组合实现
