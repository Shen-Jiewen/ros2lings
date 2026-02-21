# 提示 3

完整修复：
1. 在发送请求前加上：
   ```cpp
   if (!client->wait_for_service(5s)) {
     RCLCPP_ERROR(node->get_logger(), "Service not available");
     return 1;
   }
   ```
2. 修复异步调用——去掉 `.get()`：
   ```cpp
   auto future = client->async_send_request(request);
   ```
3. 在 `future.get()` 前加上：
   ```cpp
   rclcpp::spin_until_future_complete(node, future);
   ```
