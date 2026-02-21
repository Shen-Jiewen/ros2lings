# 节点（Node）— ROS2 的基本计算单元

## 概念

Node 是 ROS2 中最基础的运行单元。每个节点是一个独立的功能模块，
通过话题（Topic）、服务（Service）、Action 与其他节点通信。

## 生命周期

```
rclcpp::init(argc, argv)     ← 初始化 ROS2 上下文
        │
        ▼
std::make_shared<Node>("name") ← 创建节点
        │
        ▼
  注册发布者/订阅者/服务/定时器
        │
        ▼
   rclcpp::spin(node)          ← 事件循环
        │
        ▼
  rclcpp::shutdown()           ← 清理资源
```

## 关键点

- `rclcpp::init()` 初始化全局 Context，一个进程只调用一次
- 节点用 `shared_ptr` 管理，因为 Executor 和回调都需要持有引用
- `spin()` 阻塞式事件循环；`spin_some()` 处理当前队列后返回
- `shutdown()` 清理所有节点和 DDS 资源
