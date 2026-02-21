# 服务客户端（Service Client）— 异步请求/等待模式

## 概念

Service Client 向 Service Server 发送请求并等待响应。
ROS2 中的服务调用是异步的，使用 `future` 机制来获取结果。

## 异步调用流程

```
create_client<SrvType>("服务名")         ← 创建客户端
        │
        ▼
client->wait_for_service(timeout)       ← 等待服务可用
        │
        ▼
client->async_send_request(request)     ← 发送请求，返回 future
        │
        ▼
spin_until_future_complete(node, future) ← 等待响应
        │
        ▼
future.get()                            ← 获取结果
```

## 为什么需要 wait_for_service？

服务器可能还没启动，或者网络发现需要时间。
如果不等待直接发送请求，可能会失败或超时。

```cpp
if (!client->wait_for_service(5s)) {
  RCLCPP_ERROR(node->get_logger(), "Service not available");
  return 1;
}
```

## 为什么需要 spin_until_future_complete？

`async_send_request` 返回一个 `future`，但 ROS2 的通信需要节点执行 spin
来处理消息。`spin_until_future_complete` 会持续 spin 直到收到响应。

```cpp
auto future = client->async_send_request(request);
auto status = rclcpp::spin_until_future_complete(node, future, 5s);
if (status == rclcpp::FutureReturnCode::SUCCESS) {
  auto response = future.get();
}
```

## async_send_request 的参数

`async_send_request` 接受 `std::shared_ptr<Request>`，不是裸指针：
```cpp
// 正确
auto future = client->async_send_request(request);

// 错误 — request.get() 返回裸指针
auto future = client->async_send_request(request.get());
```

## 关键点

- 客户端调用是异步的，需要 `spin` 来处理通信
- `wait_for_service` 防止在服务不可用时发送请求
- `spin_until_future_complete` 是阻塞式等待，适合简单场景
- 在回调中不能使用 `spin_until_future_complete`（会死锁），需要用回调版本
