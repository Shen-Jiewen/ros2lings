# 提示 3

完整的修复：

1. 替换时间戳查询参数：
   ```cpp
   // 删除以下两行:
   // auto future_time = std::chrono::system_clock::now() + std::chrono::seconds(10);
   // auto query_time = tf2::TimePoint(...);

   // 直接在 lookupTransform 中使用 tf2::TimePointZero
   ```

2. 修复超时时间：
   ```cpp
   auto timeout = rclcpp::Duration::from_seconds(1.0);  // 从 0.0 改为 1.0
   ```

3. 修复异常捕获类型：
   ```cpp
   // 将
   } catch (const std::runtime_error & ex) {
   // 改为
   } catch (const tf2::TransformException & ex) {
   ```

修复后的 `timer_callback`：
```cpp
void timer_callback()
{
  auto timeout = rclcpp::Duration::from_seconds(1.0);

  try {
    geometry_msgs::msg::TransformStamped t;
    t = tf_buffer_->lookupTransform(
      "world",
      "robot",
      tf2::TimePointZero,
      timeout.to_chrono<std::chrono::nanoseconds>());

    RCLCPP_INFO(this->get_logger(),
      "变换 world -> robot: [%.2f, %.2f, %.2f]",
      t.transform.translation.x,
      t.transform.translation.y,
      t.transform.translation.z);

  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "查询失败: %s", ex.what());
  }
}
```
