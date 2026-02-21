# 提示 2

- 创建服务：
  ```cpp
  service_ = create_service<AddTwoInts>("add_two_ints",
    std::bind(&ServerNode::handle_add, this, std::placeholders::_1, std::placeholders::_2));
  ```
- 回调中：`response->sum = request->a + request->b;`
- 创建客户端：`client_node->create_client<AddTwoInts>("add_two_ints")`
- spin 的是 server_node（因为回调要在服务器上执行）
