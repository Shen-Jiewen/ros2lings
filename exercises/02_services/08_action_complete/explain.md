# 完整 Action — 反馈与取消

## 概念

一个完整的 Action Server 不仅要计算结果，还要:
1. 在执行过程中发布反馈，让 Client 知道进度
2. 支持取消正在执行的目标

## 执行流程

```
execute(goal_handle) {
  for each step:
    ├── 检查 is_canceling()?
    │   └── YES: canceled(result) → return
    ├── 计算下一步
    └── publish_feedback(feedback)

  succeed(result)
}
```

## 反馈发布

```cpp
auto feedback = std::make_shared<Fibonacci::Feedback>();
feedback->partial_sequence.push_back(next_value);
goal_handle->publish_feedback(feedback);
```

反馈消息会被 Client 的 `feedback_callback` 接收。

## 取消处理

```cpp
if (goal_handle->is_canceling()) {
  result->sequence = partial_sequence;
  goal_handle->canceled(result);  // 标记目标为已取消
  return;
}
```

## GoalHandle 的状态方法

| 方法 | 说明 |
|------|------|
| `succeed(result)` | 标记目标成功完成 |
| `canceled(result)` | 标记目标已取消 |
| `abort(result)` | 标记目标执行失败 |
| `is_canceling()` | 检查是否有取消请求 |
| `publish_feedback(fb)` | 发布中间反馈 |

## 关键点

- 反馈应该在循环中每一步都发布，让 Client 跟踪进度
- 取消检查必须在循环内部，否则无法响应取消请求
- `canceled()` 和 `succeed()` 只能调用一次
- 执行线程应该在 `handle_accepted` 中启动
