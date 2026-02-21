# 提示 2

- `create_subscription<MsgType>("话题名", QoS深度, 回调)` — 需要三个参数
- 回调函数的参数类型应该是 `const std_msgs::msg::String::SharedPtr` 而不是值传递
- 使用 `SharedPtr` 后，访问成员要用 `->` 而不是 `.`
- 话题名称要和发布者一致：`"chatter"`
