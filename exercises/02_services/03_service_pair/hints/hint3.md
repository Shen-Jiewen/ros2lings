# 提示 3

完整实现：

ServerNode 构造函数中：
```cpp
service_ = create_service<AddTwoInts>("add_two_ints",
  std::bind(&ServerNode::handle_add, this,
            std::placeholders::_1, std::placeholders::_2));
```

handle_add 回调：
```cpp
response->sum = request->a + request->b;
RCLCPP_INFO(get_logger(), "%ld + %ld = %ld", request->a, request->b, response->sum);
```

main 函数中：
```cpp
auto client = client_node->create_client<AddTwoInts>("add_two_ints");
client->wait_for_service(5s);
auto future = client->async_send_request(request);
rclcpp::spin_until_future_complete(server_node, future);
auto response = future.get();
RCLCPP_INFO(client_node->get_logger(), "Result: %ld", response->sum);
```
