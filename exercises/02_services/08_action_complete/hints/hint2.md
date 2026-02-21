# 提示 2

循环的结构应该是:
```cpp
for (int i = 2; i < goal->order; ++i) {
  // 1. 检查取消: if (goal_handle->is_canceling()) { ... }
  // 2. 计算: partial_sequence.push_back(partial_sequence[i-1] + partial_sequence[i-2])
  // 3. 发布反馈: goal_handle->publish_feedback(feedback)
}
```

取消处理:
```cpp
result->sequence = partial_sequence;
goal_handle->canceled(result);
return;
```
