# TF2 变换监听 — Buffer 与 TransformListener

## 概念

在 TF2 系统中，**TransformListener** 负责从 `/tf` 和 `/tf_static` 话题
订阅变换数据，并将其存入 **Buffer**（缓冲区）中。
当你需要查询两个坐标帧之间的关系时，通过 Buffer 的 `lookupTransform` 方法即可获取。

## 核心组件

### tf2_ros::Buffer
- 存储所有已知的坐标变换关系
- 提供 `lookupTransform()` 方法用于查询
- 创建时**必须传入节点的时钟**

### tf2_ros::TransformListener
- 自动订阅 `/tf` 和 `/tf_static` 话题
- 将收到的变换数据写入 Buffer
- 创建时需要传入 Buffer 的引用

## 基本用法

```cpp
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>

class TfListener : public rclcpp::Node
{
public:
  TfListener() : Node("tf_listener")
  {
    // 创建 Buffer — 必须传入时钟！
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());

    // 创建 TransformListener
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // 定时查询变换
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&TfListener::timer_callback, this));
  }

private:
  void timer_callback()
  {
    try {
      // lookupTransform(target_frame, source_frame, time)
      auto t = tf_buffer_->lookupTransform(
        "base_link",       // 目标帧
        "sensor_link",     // 源帧
        tf2::TimePointZero);  // 最新可用时间

      RCLCPP_INFO(this->get_logger(), "x=%.2f, y=%.2f, z=%.2f",
        t.transform.translation.x,
        t.transform.translation.y,
        t.transform.translation.z);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "变换查询失败: %s", ex.what());
    }
  }

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
};
```

## 关键点

### Buffer 必须传入时钟
```cpp
// 错误 — 不传时钟会导致时间查询异常
auto buffer = std::make_shared<tf2_ros::Buffer>();

// 正确
auto buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
```

### lookupTransform 参数顺序
```cpp
// lookupTransform(target_frame, source_frame, time)
// 含义：查询 source_frame 在 target_frame 坐标系中的位姿
auto t = buffer->lookupTransform("base_link", "sensor_link", tf2::TimePointZero);
// 结果的 header.frame_id 是 "base_link"
// 结果的 child_frame_id 是 "sensor_link"
```

**注意参数顺序！** 第一个参数是目标帧（target），第二个是源帧（source）。
这和 "从 A 到 B 的变换" 的直觉顺序可能相反。

### 异常处理不可省略
`lookupTransform` 在以下情况会抛出异常：
- `tf2::LookupException` — 请求的帧不存在
- `tf2::ConnectivityException` — 两个帧之间没有连接路径
- `tf2::ExtrapolationException` — 请求的时间超出缓冲区范围

所有这些异常都继承自 `tf2::TransformException`，所以通常用：
```cpp
try {
  auto t = buffer->lookupTransform(...);
} catch (const tf2::TransformException & ex) {
  RCLCPP_WARN(this->get_logger(), "%s", ex.what());
}
```
