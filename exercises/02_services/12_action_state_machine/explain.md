# Action 状态机 — 理解 Action 的生命周期

## 概念

ROS2 Action 有一个严格的状态机，每个目标都会经历以下状态转换:

```
           handle_goal()
               |
          [ACCEPTED?]
         /          \
     REJECT       ACCEPT
       |            |
       x      handle_accepted()
               |
          [启动执行线程]
               |
           EXECUTING
          /    |     \
    cancel  compute  error
      |       |       |
  CANCELING  完成    ABORTED
      |       |
  CANCELED  SUCCEEDED
```

## 常见的状态转换错误

### 错误 1: 目标接受但不执行

```cpp
// 错误: handle_accepted 什么都不做
void handle_accepted(goal_handle) {
  // 目标卡在 ACCEPTED 状态，永远不执行
}

// 正确: 启动执行线程
void handle_accepted(goal_handle) {
  std::thread{...execute...}.detach();
}
```

### 错误 2: 拒绝所有取消请求

```cpp
// 错误: 客户端无法取消任务
return CancelResponse::REJECT;

// 正确: 接受取消请求
return CancelResponse::ACCEPT;
```

### 错误 3: 过早设置结果

```cpp
// 错误: 在计算完成前就 succeed()
result->sequence = partial_sequence;  // 只有 [0, 1]
goal_handle->succeed(result);         // 不完整的结果!
for (...) { compute(); }              // 这些计算毫无意义

// 正确: 先完成计算，再设置结果
for (...) { compute(); }
result->sequence = partial_sequence;  // 完整的序列
goal_handle->succeed(result);         // 正确的结果
```

## GoalHandle 状态方法

| 方法 | 从状态 | 到状态 | 说明 |
|------|--------|--------|------|
| `succeed(result)` | EXECUTING | SUCCEEDED | 成功完成 |
| `canceled(result)` | CANCELING | CANCELED | 确认取消 |
| `abort(result)` | EXECUTING | ABORTED | 执行失败 |

## 关键点

- `handle_accepted` **必须** 启动执行，否则目标永远不会完成
- `handle_cancel` 返回 `REJECT` 意味着客户端无法取消任务
- `succeed()` / `canceled()` / `abort()` 只能调用一次
- 结果必须在计算完成后设置，不能提前调用 `succeed()`
- 调用 `succeed()` 后再调用 `publish_feedback()` 会导致错误
