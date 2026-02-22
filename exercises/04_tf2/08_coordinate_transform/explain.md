# TF2 坐标变换 — tf2::doTransform

## 概念

在机器人系统中，传感器数据通常在传感器自身的坐标帧中表示，但实际使用时
往往需要将这些数据转换到其他坐标帧（如地图帧或机器人本体帧）。
TF2 提供了 `tf2::doTransform` 函数来完成这个操作。

## 基本流程

坐标变换分为两步：
1. **查询变换**: 使用 `lookupTransform` 获取两个帧之间的变换关系
2. **执行变换**: 使用 `tf2::doTransform` 将数据从源帧变换到目标帧

```cpp
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

// 1. 创建 Buffer 和 TransformListener
auto buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
auto listener = std::make_shared<tf2_ros::TransformListener>(*buffer);

// 2. 创建源帧中的点
geometry_msgs::msg::PointStamped point_in;
point_in.header.frame_id = "sensor_link";
point_in.header.stamp = this->now();
point_in.point.x = 1.0;
point_in.point.y = 0.0;
point_in.point.z = 0.0;

// 3. 查询变换 (从 sensor_link 到 map)
auto transform = buffer->lookupTransform(
  "map",          // 目标帧 (target)
  "sensor_link",  // 源帧 (source)
  tf2::TimePointZero);

// 4. 执行变换
geometry_msgs::msg::PointStamped point_out;
tf2::doTransform(point_in, point_out, transform);
// point_out 现在是 map 帧中的坐标
```

## tf2_geometry_msgs

`tf2_geometry_msgs` 包提供了对常见 geometry_msgs 类型的 `doTransform` 特化，
支持的类型包括：

| 消息类型 | 说明 |
|---------|------|
| `PointStamped` | 带时间戳的三维点 |
| `Vector3Stamped` | 带时间戳的三维向量 |
| `PoseStamped` | 带时间戳的位姿 |
| `QuaternionStamped` | 带时间戳的四元数 |
| `TransformStamped` | 变换消息本身 |

使用时需要包含头文件：
```cpp
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
```

## lookupTransform 参数顺序

```cpp
buffer->lookupTransform(target_frame, source_frame, time);
```

- **target_frame**: 你想要数据所在的目标帧
- **source_frame**: 数据当前所在的源帧
- **time**: 查询时间点，`tf2::TimePointZero` 表示最新的变换

返回的变换表示"从 source 到 target 的变换"。

## 纯平移变换的直观理解

如果帧之间只有平移、没有旋转：
- `map -> sensor_link` 平移为 `(2, 0, 0)`
- `sensor_link` 中的点 `(1, 0, 0)`
- 变换到 `map` 后：`(1+2, 0, 0) = (3, 0, 0)`

## 关键点

### 异常处理
- `lookupTransform` 可能抛出 `tf2::TransformException`
- 常见原因：帧不存在、变换过期、帧链不连通
- 务必用 try-catch 包裹变换代码

### 时间戳
- `PointStamped` 等消息自带 `header.stamp`
- `lookupTransform` 可以指定查询时间
- 使用 `tf2::TimePointZero` 获取最新可用的变换
