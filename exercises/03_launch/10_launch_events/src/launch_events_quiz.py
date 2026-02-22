#!/usr/bin/env python3
# I AM NOT DONE
#
# 练习: launch_events
# 模块: 03 - Launch & Parameters
# 难度: ★★★☆☆
#
# 学习目标:
#   了解 ROS2 Launch 事件系统，掌握事件驱动的 Launch 控制。
#
# 说明:
#   这是一个"探索"类练习。根据你对 ROS2 Launch 事件系统的了解，
#   填写下面每个问答变量的正确答案。
#
#   你可以参考以下资料：
#   - ROS2 Launch 文档: https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Using-Event-Handlers.html
#   - launch 包源码中的 events 和 event_handlers 模块
#
# 步骤:
#   1. 根据 ROS2 知识填写每个 answer 变量的正确值
#   2. 填写完成后，删除文件顶部的 "# I AM NOT DONE"

# ===== 问答区域 =====

# 问题 1: 当一个进程退出后，哪个 launch 事件会被触发？
# 选项: "OnProcessStart" / "OnProcessExit" / "OnShutdown"
answer_process_exit_event = "FILL_IN"

# 问题 2: 哪个类用于注册事件处理器？
# 选项: "EventHandler" / "RegisterEventHandler" / "AddAction"
answer_register_handler = "FILL_IN"

# 问题 3: launch 是否支持"先启动 A，A 退出后再启动 B"的模式？
# 选项: "yes" / "no"
answer_chained_launch = "FILL_IN"

# 问题 4: 哪个 action 可以在事件触发时关闭整个 launch？
# 选项: "Shutdown" / "Exit" / "Kill"
answer_shutdown_action = "FILL_IN"

# 问题 5: OnProcessExit 事件可以访问进程的退出码吗？
# 选项: "yes" / "no"
answer_exit_code_access = "FILL_IN"

# ===== 结束 =====

if __name__ == '__main__':
    print(f'answer_process_exit_event = {answer_process_exit_event}')
    print(f'answer_register_handler = {answer_register_handler}')
    print(f'answer_chained_launch = {answer_chained_launch}')
    print(f'answer_shutdown_action = {answer_shutdown_action}')
    print(f'answer_exit_code_access = {answer_exit_code_access}')
