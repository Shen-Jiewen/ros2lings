# Publisher（发布者）— 向话题发送消息

## 概念

Publisher 是 ROS2 中向话题（Topic）发送消息的组件。
一个节点可以创建多个 Publisher，向不同的话题发布不同类型的消息。

## 创建流程

```
create_publisher<MsgType>("话题名", QoS深度)   ← 创建发布者
        │
        ▼
create_wall_timer(周期, 回调函数)              ← 创建定时器
        │
        ▼
timer_callback() {                            ← 定时回调
  publisher_->publish(message);               ← 发布消息
}
```

## 关键点

- `create_publisher` 是模板函数，需要用 `<>` 指定消息类型
- 第二个参数是 QoS（Quality of Service）队列深度，通常设为 10
- `create_wall_timer` 的周期参数需要 `std::chrono` 类型（如 `500ms`）
- `publish()` 将消息发送到话题，所有订阅者都会收到
- 使用 `using namespace std::chrono_literals;` 可以直接写 `500ms`
