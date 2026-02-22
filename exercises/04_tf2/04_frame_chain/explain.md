# TF2 多帧链 — Frame Chain

## 概念

在真实的机器人系统中，坐标帧之间通常形成一条**链**（chain），而不仅仅是单一的父子关系。
TF2 会自动将链上的多个变换**级联**（compose），让你可以直接查询链两端之间的关系。

典型的帧链示例：

```
map -> odom -> base_link -> sensor_link
```

- **map**: 全局参考帧（世界坐标系）
- **odom**: 里程计帧（累积漂移的定位结果）
- **base_link**: 机器人本体中心
- **sensor_link**: 传感器安装位置

## 一次发布多个变换

`StaticTransformBroadcaster` 的 `sendTransform` 方法可以接受一个
`std::vector<geometry_msgs::msg::TransformStamped>`，一次发布多个变换：

```cpp
#include <tf2_ros/static_transform_broadcaster.h>
#include <vector>

std::vector<geometry_msgs::msg::TransformStamped> transforms;

geometry_msgs::msg::TransformStamped t1;
t1.header.stamp = this->now();
t1.header.frame_id = "map";
t1.child_frame_id = "odom";
t1.transform.translation.x = 1.0;
t1.transform.rotation.w = 1.0;
transforms.push_back(t1);

geometry_msgs::msg::TransformStamped t2;
t2.header.stamp = this->now();
t2.header.frame_id = "odom";
t2.child_frame_id = "base_link";
t2.transform.translation.x = 0.5;
t2.transform.rotation.w = 1.0;
transforms.push_back(t2);

// 一次发布所有变换
broadcaster->sendTransform(transforms);
```

## 帧链的查询

当 TF2 buffer 中存在 `A -> B` 和 `B -> C` 的变换时，
你可以直接查询 `A -> C`，TF2 会自动把中间的变换级联起来：

```cpp
// 查询 sensor_link 在 map 坐标系下的位姿
auto t = buffer->lookupTransform("map", "sensor_link", tf2::TimePointZero);
// 得到的平移 = map->odom 的平移 + odom->base_link 的平移 + base_link->sensor_link 的平移
```

## 关键点

### 帧树规则
- TF2 的帧关系形成一棵**树**（tree），不是图（graph）
- 每个子帧**只能有一个父帧**
- 不允许出现环（loop），否则查询会失败

### 变换级联
- 链上的变换按顺序**相乘**（矩阵乘法 / 四元数组合）
- 纯平移的级联就是向量加法
- 有旋转时需要先旋转再平移

### 常见帧名约定
| 帧名 | 含义 |
|------|------|
| `map` | 全局固定帧，无漂移 |
| `odom` | 里程计帧，可能有累积漂移 |
| `base_link` | 机器人本体中心 |
| `base_footprint` | 机器人在地面的投影 |
| `sensor_link` | 传感器安装位置 |
