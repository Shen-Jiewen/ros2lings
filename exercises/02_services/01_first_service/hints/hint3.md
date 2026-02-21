# 提示 3

完整修复：
1. 服务名改为 `"add_two_ints"`
2. 回调签名加上 response 参数：
   ```cpp
   void handle_add(
     const std::shared_ptr<ros2lings_interfaces::srv::AddTwoInts::Request> request,
     std::shared_ptr<ros2lings_interfaces::srv::AddTwoInts::Response> response)
   ```
3. 将 `response->result` 改为 `response->sum`
