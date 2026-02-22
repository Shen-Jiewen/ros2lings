# 提示 3

完整的修复：

1. 将广播器类型从 Static 改为动态：
   - 构造函数中：
     ```cpp
     tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
     ```
   - 成员变量声明：
     ```cpp
     std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
     ```

2. 在 `broadcast_timer_callback()` 中添加时间戳更新：
   ```cpp
   t.header.stamp = this->now();
   ```

3. 取消注释 `sendTransform` 调用：
   ```cpp
   tf_broadcaster_->sendTransform(t);
   ```
