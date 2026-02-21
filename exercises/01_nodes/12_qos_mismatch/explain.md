# QoS（服务质量）— 控制通信行为的策略

## 概念

QoS（Quality of Service）是 DDS 中间件的核心概念，允许你精确控制
话题通信的行为，包括可靠性、持久性、历史记录、截止期限等。

## 关键策略

### Reliability（可靠性）
- `Reliable`: 保证消息送达，会重传丢失的消息
- `BestEffort`: 尽力传递，不保证送达，适合高频传感器数据

### Durability（持久性）
- `TransientLocal`: 为后来的订阅者缓存最近的消息
- `Volatile`: 不缓存，订阅者只收到订阅后发布的消息

## 兼容性规则

```
发布者          订阅者          结果
─────────      ─────────      ────
Reliable    +  Reliable    =  兼容 ✓
Reliable    +  BestEffort  =  兼容 ✓
BestEffort  +  Reliable    =  不兼容 ✗
BestEffort  +  BestEffort  =  兼容 ✓

TransientLocal + TransientLocal = 兼容 ✓
TransientLocal + Volatile       = 兼容 ✓
Volatile       + TransientLocal = 不兼容 ✗
Volatile       + Volatile       = 兼容 ✓
```

## 调试技巧

- 使用 `ros2 topic info -v <topic>` 查看 QoS 详情
- 不兼容时 ROS2 会在日志中输出警告（但容易被忽略）
- 原则：发布者的 QoS 必须 **至少** 满足订阅者的要求
