# 提示 3

完整的修复：

1. 交换 robot_name 的声明和获取顺序：
   ```python
   self.declare_parameter('robot_name', 'default_bot')
   name = self.get_parameter('robot_name').value
   ```

2. 修复 max_count 的默认值：
   ```python
   self.declare_parameter('max_count', 10)
   ```

3. 取消注释 timer_period 的声明和获取：
   ```python
   self.declare_parameter('timer_period', 1.0)
   period = self.get_parameter('timer_period').value
   self.timer_period_ = period
   ```

4. timer 使用参数值：
   ```python
   self.timer_ = self.create_timer(self.timer_period_, self.timer_callback)
   ```
