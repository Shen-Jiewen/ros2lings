# 提示 3

完整的修复：

1. 修改导入语句：
   ```python
   from tf2_ros import TransformBroadcaster
   ```
   同时修改构造函数中的创建代码：
   ```python
   self.broadcaster_ = TransformBroadcaster(self)
   ```

2. 在 `timer_callback` 中添加时间戳设置：
   ```python
   t.header.stamp = self.get_clock().now().to_msg()
   ```

3. 交换 `frame_id` 和 `child_frame_id` 的值：
   ```python
   t.header.frame_id = 'world'
   t.child_frame_id = 'child_frame'
   ```
