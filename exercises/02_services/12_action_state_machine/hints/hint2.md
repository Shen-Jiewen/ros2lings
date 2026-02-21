# 提示 2

三个具体的修复:

1. `handle_cancel`: 将 `REJECT` 改为 `ACCEPT`
2. `handle_accepted`: 添加 `std::thread{std::bind(&FibonacciStateMachine::execute, this, goal_handle)}.detach();`
3. `execute`: 将 `succeed()` 和结果赋值移到 for 循环 **之后**
