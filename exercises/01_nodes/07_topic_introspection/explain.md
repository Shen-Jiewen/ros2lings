# 话题内省 — 用 CLI 工具调试 ROS2 系统

## 概念

ROS2 提供了一套命令行工具来查看和调试正在运行的系统。
这些工具对于理解系统架构和排查问题非常有用。

## 常用命令

```bash
# 列出所有活跃话题
ros2 topic list

# 查看话题详细信息（发布者/订阅者数量）
ros2 topic info /chatter

# 查看话题的消息类型
ros2 topic type /chatter

# 实时查看话题上的消息
ros2 topic echo /chatter

# 查看话题发布频率
ros2 topic hz /chatter

# 从命令行发布消息
ros2 topic pub /chatter std_msgs/msg/String "data: 'hello'"

# 查看消息类型定义
ros2 interface show std_msgs/msg/String
```

## 关键点

- 话题只在有活跃的发布者或订阅者时才会出现在 `topic list` 中
- `ros2 topic echo` 会自动创建一个订阅者
- `ros2 topic info -v` 可以查看 QoS 详细配置
- `ros2 interface show` 可以查看任何消息类型的字段定义
- 这些工具是调试 ROS2 系统的必备技能
