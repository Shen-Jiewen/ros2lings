# 多节点进程 — Executor 与进程内通信

## 概念

ROS2 允许在单个进程中运行多个节点。通过 Executor（执行器）统一管理
这些节点的回调函数调度。

## Executor 类型

### SingleThreadedExecutor
```cpp
rclcpp::executors::SingleThreadedExecutor executor;
```
- 单线程串行处理所有节点的回调
- 简单、可预测，适合大多数场景
- 一个回调阻塞会影响所有节点

### MultiThreadedExecutor
```cpp
rclcpp::executors::MultiThreadedExecutor executor;
```
- 多线程并行处理回调
- 适合有耗时回调的场景
- 需要注意线程安全

## 使用模式

```cpp
// 创建多个节点
auto node_a = std::make_shared<NodeA>();
auto node_b = std::make_shared<NodeB>();

// 添加到同一个 executor
rclcpp::executors::MultiThreadedExecutor executor;
executor.add_node(node_a);
executor.add_node(node_b);

// 统一运行
executor.spin();
```

## 与多进程的对比

| 特性        | 多节点单进程         | 单节点多进程         |
|------------|--------------------|--------------------|
| 通信效率    | 高（可进程内零拷贝）  | 低（经过 DDS）      |
| 隔离性      | 低（共享内存空间）    | 高（进程隔离）       |
| 资源开销    | 低                  | 高                  |
| 调试难度    | 较高                | 较低                |

## 进程内通信

当发布者和订阅者在同一进程中时，ROS2 可以使用进程内通信
（Intra-process Communication），实现零拷贝消息传递，
大幅提升性能。
