# 提示 3

完整的修复：

1. 修复 TransformListener 的创建，传入 buffer：
   ```python
   self.listener_ = TransformListener(self.buffer_, self)
   ```

2. 修复 lookup_transform 的帧参数顺序：
   ```python
   t = self.buffer_.lookup_transform(
       self.target_frame_,   # 'world' — 目标帧
       self.source_frame_,   # 'child_frame' — 源帧
       rclpy.time.Time())
   ```

3. 修复异常处理类型：
   ```python
   except tf2_ros.TransformException as ex:
       self.get_logger().warn(f'无法获取变换: {ex}')
   ```
