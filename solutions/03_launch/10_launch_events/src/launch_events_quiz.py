#!/usr/bin/env python3

# ===== 问答区域 =====

# 问题 1: 当一个进程退出后，哪个 launch 事件会被触发？
answer_process_exit_event = "OnProcessExit"

# 问题 2: 哪个类用于注册事件处理器？
answer_register_handler = "RegisterEventHandler"

# 问题 3: launch 是否支持"先启动 A，A 退出后再启动 B"的模式？
answer_chained_launch = "yes"

# 问题 4: 哪个 action 可以在事件触发时关闭整个 launch？
answer_shutdown_action = "Shutdown"

# 问题 5: OnProcessExit 事件可以访问进程的退出码吗？
answer_exit_code_access = "yes"

# ===== 结束 =====

if __name__ == '__main__':
    print(f'answer_process_exit_event = {answer_process_exit_event}')
    print(f'answer_register_handler = {answer_register_handler}')
    print(f'answer_chained_launch = {answer_chained_launch}')
    print(f'answer_shutdown_action = {answer_shutdown_action}')
    print(f'answer_exit_code_access = {answer_exit_code_access}')
