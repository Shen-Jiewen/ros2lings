# Subscription（订阅者）— 从话题接收消息

## 概念

Subscription 是 ROS2 中从话题接收消息的组件。
当有新消息发布到订阅的话题时，回调函数会被自动调用。

## 创建流程

```
create_subscription<MsgType>("话题名", QoS, 回调)  ← 创建订阅
        │
        ▼
  rclcpp::spin(node)                              ← 事件循环等待消息
        │
        ▼
  topic_callback(msg)                             ← 收到消息时触发回调
```

## 关键点

- 回调函数参数必须是 `SharedPtr` 类型（如 `const String::SharedPtr msg`）
- QoS（Quality of Service）控制消息队列深度和可靠性
- 话题名称必须和发布者完全匹配才能通信
- `spin()` 是阻塞式的，节点会一直等待并处理消息
- 使用 `std::bind` 或 lambda 绑定类成员回调
