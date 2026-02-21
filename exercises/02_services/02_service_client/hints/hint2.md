# 提示 2

- 等待服务：`client->wait_for_service(5s)` — 返回 `true` 表示服务可用
- 异步发送请求：`client->async_send_request(request)` — 参数应该是 `shared_ptr`，不是裸指针
- 等待结果：`rclcpp::spin_until_future_complete(node, future)` — 在 `future.get()` 之前调用
