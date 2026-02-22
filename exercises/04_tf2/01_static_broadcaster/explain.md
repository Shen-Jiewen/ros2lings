# TF2 静态坐标变换 — StaticTransformBroadcaster

## 概念

TF2 是 ROS2 中的坐标变换系统，它维护着一棵坐标帧（frame）树，
让你可以随时查询任意两个帧之间的空间关系（位置和姿态）。

**静态变换**（Static Transform）用于描述不会随时间变化的空间关系，
例如传感器相对于机器人本体的固定安装位置。

## StaticTransformBroadcaster

```cpp
#include <tf2_ros/static_transform_broadcaster.h>

// 在节点构造函数中创建
auto broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

// 构建变换消息
geometry_msgs::msg::TransformStamped t;
t.header.stamp = this->now();
t.header.frame_id = "world";       // 父帧
t.child_frame_id = "base_link";    // 子帧

// 设置平移
t.transform.translation.x = 1.0;
t.transform.translation.y = 0.0;
t.transform.translation.z = 0.0;

// 设置旋转（单位四元数 = 无旋转）
t.transform.rotation.x = 0.0;
t.transform.rotation.y = 0.0;
t.transform.rotation.z = 0.0;
t.transform.rotation.w = 1.0;

// 发布变换
broadcaster->sendTransform(t);
```

## TransformStamped 消息结构

```
geometry_msgs/msg/TransformStamped:
  header:
    stamp: 时间戳
    frame_id: 父帧名称
  child_frame_id: 子帧名称
  transform:
    translation: {x, y, z}   # 平移
    rotation: {x, y, z, w}   # 旋转（四元数）
```

## 关键概念

### 帧的父子关系
- `header.frame_id` 是**父帧**（参考帧），如 "world"
- `child_frame_id` 是**子帧**（被描述的帧），如 "base_link"
- 变换描述的是：子帧相对于父帧的位置和姿态

### 四元数基础
- 四元数 `(x, y, z, w)` 用于表示三维旋转
- **单位四元数** `(0, 0, 0, 1)` 表示无旋转（恒等旋转）
- 四元数的模（norm）必须为 1：`x^2 + y^2 + z^2 + w^2 = 1`
- `w = 0, x = y = z = 0` 不是有效的四元数！

### 静态 vs 动态变换
- 静态变换只需发布一次，TF2 会持久化记忆
- 动态变换需要以一定频率持续发布
- 使用 `StaticTransformBroadcaster` 发布静态变换
- 使用 `TransformBroadcaster` 发布动态变换
