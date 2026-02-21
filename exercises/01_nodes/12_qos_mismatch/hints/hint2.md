# 提示 2

这里有两个不兼容问题：
1. 发布者是 `BestEffort`，订阅者是 `Reliable` — 不兼容！
   - `BestEffort` 发布者无法满足 `Reliable` 订阅者的确认要求
2. 发布者是 `Volatile`，订阅者是 `TransientLocal` — 不兼容！
   - `Volatile` 发布者不缓存消息，无法满足 `TransientLocal` 订阅者的期望

最简单的修复：让发布者也使用 `Reliable` 和 `TransientLocal`。
