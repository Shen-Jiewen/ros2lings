# 提示 2

- 导入路径：`ros2lings_interfaces`（复数 s），不是 `example_interface`
- 回调函数签名：`def handle_add(self, request, response):` — 别忘了 `self`
- 服务名不匹配：服务器用了 `'add_numbers'`，客户端用了 `'add_two_ints'`，需要统一
