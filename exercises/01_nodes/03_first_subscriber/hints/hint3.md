# 提示 3

完整的修复：
1. 话题名从 `"wrong_topic"` 改为 `"chatter"`
2. 添加 QoS 参数：`"chatter", 10, std::bind(...)`
3. 回调参数改为：`const std_msgs::msg::String::SharedPtr msg`
4. 回调中用 `msg->data` 替代 `msg.data`
