# 提示 3

完整修复：
1. 导入语句改为：
   ```python
   from ros2lings_interfaces.srv import AddTwoInts
   ```
2. 回调签名加上 self：
   ```python
   def handle_add(self, request, response):
   ```
3. 服务名统一为 `'add_two_ints'`：
   ```python
   self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.handle_add)
   ```
