# 提示 2

三个错误分别是：

1. `get_parameter('robot_name')` 在 `declare_parameter('robot_name', ...)` 之前调用了，
   需要交换顺序，先 declare 再 get。

2. `self.declare_parameter('max_count', 'ten')` — `'ten'` 是字符串，
   应改为整数 `10`。

3. `timer_period` 参数的声明和获取被注释掉了，需要取消注释：
   ```python
   self.declare_parameter('timer_period', 1.0)
   period = self.get_parameter('timer_period').value
   ```
