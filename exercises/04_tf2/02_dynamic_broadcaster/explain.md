# TF2 动态坐标变换 — TransformBroadcaster

## 概念

与静态变换不同，**动态变换**描述的是随时间变化的空间关系。
例如一个旋转的雷达、运动中的机械臂关节、或者移动的机器人本体。

动态变换需要以一定频率**持续发布**，通常在定时器回调函数中完成。

## TransformBroadcaster vs StaticTransformBroadcaster

| 特性 | TransformBroadcaster | StaticTransformBroadcaster |
|------|---------------------|---------------------------|
| 话题 | `/tf` | `/tf_static` |
| 发布频率 | 持续发布（如 10~100Hz） | 只需发布一次 |
| 适用场景 | 运动关节、移动平台 | 传感器安装位置等固定关系 |
| QoS | 普通可靠性 | 持久化（latched） |

## 基本用法

```cpp
#include <tf2_ros/transform_broadcaster.h>

class DynamicBroadcaster : public rclcpp::Node
{
public:
  DynamicBroadcaster() : Node("dynamic_broadcaster")
  {
    // 创建动态变换广播器
    broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // 创建定时器，每 100ms 发布一次
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&DynamicBroadcaster::timer_callback, this));
  }

private:
  void timer_callback()
  {
    geometry_msgs::msg::TransformStamped t;

    // 关键：每次回调都要更新时间戳！
    t.header.stamp = this->now();

    t.header.frame_id = "base_link";
    t.child_frame_id = "sensor_link";

    // 设置随时间变化的平移/旋转
    double secs = this->now().seconds();
    t.transform.translation.x = std::cos(secs);
    // ...

    // 发布变换
    broadcaster_->sendTransform(t);
  }

  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};
```

## 关键点

### 时间戳必须更新
每次发布动态变换时，必须将 `header.stamp` 设置为当前时间 `this->now()`。
如果时间戳不更新，TF2 的查询（lookupTransform）会因为时间不匹配而失败。

### 不要混淆广播器类型
- 对持续变化的关系使用 `TransformBroadcaster`
- 对固定不变的关系使用 `StaticTransformBroadcaster`
- 用错了不会直接报错，但会导致行为异常：
  - 把动态变换用 Static 发布 → 只有最初的那个值被记住，后续更新被忽略
  - 把静态变换用动态发布 → 浪费带宽，且超时后变换会"消失"

### sendTransform 必须调用
构建了 `TransformStamped` 消息之后，必须调用 `sendTransform()` 才能实际发布。
忘记调用是一个常见的疏忽。
