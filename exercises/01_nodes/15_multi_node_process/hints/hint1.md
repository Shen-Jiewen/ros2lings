# 提示 1

要在一个进程中运行多个节点，你需要：
1. 创建每个节点的 `shared_ptr` 实例
2. 使用一个 `Executor` 来管理它们

`rclcpp::executors` 命名空间下有哪些 Executor 可用？
