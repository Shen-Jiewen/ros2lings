# 提示 2

使用 `MultiThreadedExecutor` 的步骤：

```cpp
// 1. 创建 executor
rclcpp::executors::MultiThreadedExecutor executor;

// 2. 添加节点
executor.add_node(node_ptr);

// 3. 运行
executor.spin();  // 阻塞式，处理所有节点的回调
```

别忘了先用 `std::make_shared` 创建两个节点实例！
