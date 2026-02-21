# Action Client — 发送目标并处理反馈

## 概念

Action Client 用于向 Action Server 发送目标请求，并处理执行过程中的反馈和最终结果。
与 Service Client 不同，Action Client 可以在任务执行期间持续接收进度反馈。

## 通信流程

```
Client                        Server
  |                             |
  |--- send_goal(order=10) ---> |
  |                             |
  |<-- goal_response(accepted)--|
  |                             |
  |<-- feedback(seq=[0,1,1]) ---|  (多次)
  |<-- feedback(seq=[0,1,1,2])--|
  |                             |
  |<-- result(seq=[0,1,...,55])-|
```

## SendGoalOptions 三个回调

| 回调 | 触发时机 | 签名 |
|------|----------|------|
| `goal_response_callback` | Server 接受/拒绝目标 | `(GoalHandle::SharedPtr &)` |
| `feedback_callback` | Server 发送进度反馈 | `(GoalHandle::SharedPtr, Feedback::ConstSharedPtr)` |
| `result_callback` | 任务完成/取消/中止 | `(GoalHandle::WrappedResult &)` |

## 关键点

- `async_send_goal` 需要 `goal_msg` 和 `send_goal_options` 两个参数
- 忘记传 `send_goal_options` 会导致收不到反馈和结果
- `ResultCode` 有三种: `SUCCEEDED`, `ABORTED`, `CANCELED`
- Action Client 是异步的，需要 `spin` 来处理回调
