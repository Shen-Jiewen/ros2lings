# 提示 2

三个错误分别是：

1. `TransformListener` 创建时没有传入 `buffer_`：
   ```python
   # 错误
   self.listener_ = TransformListener(node=self)
   # 正确
   self.listener_ = TransformListener(self.buffer_, self)
   ```

2. `lookup_transform` 的帧参数反了：
   - 第一个参数应该是 `target_frame`（目标帧）
   - 第二个参数应该是 `source_frame`（源帧）
   - 当前代码中 source 和 target 的位置对调了

3. 异常处理捕获了 `ValueError`，但 TF2 抛出的异常是
   `tf2_ros.TransformException`（或其子类如 `LookupException`）。
