# 提示 1

ROS2 Launch 事件系统中，有几个关键的事件和操作类：

- 进程退出相关的事件处理器是 `OnProcess___`（Start / Exit）
- 注册事件处理器的类名是描述这个动作的词组
- Launch 确实支持"先后顺序启动"模式

查看 `launch.event_handlers` 和 `launch.actions` 模块了解更多。
