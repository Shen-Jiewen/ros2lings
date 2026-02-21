# 提示 2

- 回调签名应该是：
  `void handle_add(const std::shared_ptr<Request> request, std::shared_ptr<Response> response)`
- 服务名称不能有空格，应该用下划线：`"add_two_ints"`
- `AddTwoInts` 的 Response 字段是 `sum`，不是 `result`
