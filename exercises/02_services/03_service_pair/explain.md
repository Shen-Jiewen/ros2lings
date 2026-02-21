# 服务配对（Service Pair）— 完整的请求/应答流程

## 概念

在实际的 ROS2 系统中，Service Server 和 Client 通常在不同的节点甚至不同的进程中。
这个练习将两者放在同一个文件中，帮助你理解完整的通信流程。

## 完整流程图

```
 Client Node                     Server Node
     │                               │
     │  create_client<SrvType>       │  create_service<SrvType>
     │                               │
     │  wait_for_service(timeout) ──→│  （服务注册到 DDS）
     │         ← true ──────────────│
     │                               │
     │  async_send_request(req) ────→│  handle_callback(req, res)
     │                               │    res->sum = req->a + req->b
     │  spin_until_future_complete   │
     │         ← response ─────────│
     │                               │
     │  future.get()                 │
```

## spin_until_future_complete 的节点选择

这是一个容易混淆的点：

```cpp
// 正确：spin server_node（回调在 server_node 上）
rclcpp::spin_until_future_complete(server_node, future);

// 如果 server 和 client 在同一个节点上：
rclcpp::spin_until_future_complete(node, future);
```

`spin_until_future_complete` 会 spin 第一个参数的节点，直到 future 完成。
因为服务回调需要 spin 才能被调用，所以必须 spin 拥有服务的节点。

## 两个节点 vs 一个节点

| 方式 | 优点 | 缺点 |
|------|------|------|
| 两个节点 | 职责清晰，符合 ROS2 设计哲学 | 代码稍多 |
| 一个节点 | 代码简洁 | spin 时会同时处理所有回调 |

## 关键点

- Server 和 Client 的服务名称必须完全一致
- `spin_until_future_complete` 需要 spin 拥有服务回调的节点
- 一个节点可以同时拥有多个 Service Server 和 Client
- 这种同步等待模式适合简单场景，复杂场景推荐使用回调
