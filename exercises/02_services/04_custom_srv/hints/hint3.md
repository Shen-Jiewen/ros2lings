# 提示 3

完整实现：

1. 头文件：
   ```cpp
   #include <ros2lings_19_custom_srv/srv/compute_area.hpp>
   ```

2. 回调函数：
   ```cpp
   response->area = request->width * request->height;
   ```

3. main 函数中取消所有 TODO 注释的代码即可：
   ```cpp
   auto client = client_node->create_client<ros2lings_19_custom_srv::srv::ComputeArea>("compute_area");
   client->wait_for_service(5s);
   auto request = std::make_shared<ros2lings_19_custom_srv::srv::ComputeArea::Request>();
   request->width = 3.5;
   request->height = 4.2;
   auto future = client->async_send_request(request);
   rclcpp::spin_until_future_complete(server_node, future);
   auto response = future.get();
   ```
