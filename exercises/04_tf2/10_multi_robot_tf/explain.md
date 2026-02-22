# 多机器人 TF 树 — Multi-Robot TF

## 概念

在多机器人系统中，每个机器人都有自己的坐标帧集合（base_link、sensor 等）。
如果所有机器人使用相同的帧名，TF 树会产生冲突（一个子帧不能有多个父帧）。
解决方案是使用**帧名前缀**（frame prefix）进行命名空间隔离。

## 多机器人 TF 树结构

典型的两个机器人共享一个世界帧的 TF 树：

```
                world
               /     \
   robot1/base_link   robot2/base_link
        |                   |
   robot1/sensor       robot2/sensor
```

每个机器人的帧都带有机器人名称前缀，这样它们可以共存于同一个 TF 树中。

## 帧名前缀策略

```cpp
// robot1 的帧
"robot1/base_link"
"robot1/sensor"

// robot2 的帧
"robot2/base_link"
"robot2/sensor"

// 共享帧（无前缀）
"world"
"map"
```

### 前缀的好处
- 避免帧名冲突
- 清晰标识帧所属的机器人
- 可以在同一个 TF 树中查询跨机器人的变换

## 一次发布多个机器人的变换

```cpp
#include <tf2_ros/static_transform_broadcaster.h>
#include <vector>

auto broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
std::vector<geometry_msgs::msg::TransformStamped> transforms;

// robot1: world -> robot1/base_link
geometry_msgs::msg::TransformStamped t1;
t1.header.stamp = this->now();
t1.header.frame_id = "world";
t1.child_frame_id = "robot1/base_link";
t1.transform.translation.x = 1.0;
t1.transform.rotation.w = 1.0;
transforms.push_back(t1);

// robot1: robot1/base_link -> robot1/sensor
geometry_msgs::msg::TransformStamped t2;
t2.header.stamp = this->now();
t2.header.frame_id = "robot1/base_link";
t2.child_frame_id = "robot1/sensor";
t2.transform.translation.z = 0.5;
t2.transform.rotation.w = 1.0;
transforms.push_back(t2);

// 类似地为 robot2 创建变换...

broadcaster->sendTransform(transforms);
```

## 跨机器人查询

因为两个机器人共享 `world` 帧，你可以查询任意两个帧之间的变换：

```cpp
// 查询 robot1 传感器相对于 robot2 传感器的位置
auto t = buffer->lookupTransform("robot2/sensor", "robot1/sensor", tf2::TimePointZero);
```

TF2 会自动沿着帧链查找路径：
```
robot1/sensor -> robot1/base_link -> world -> robot2/base_link -> robot2/sensor
```

## 关键点

### 命名约定
- 使用 `/` 分隔前缀和帧名：`robot1/base_link`
- 共享帧（如 `world`、`map`）不加前缀
- 前缀通常与 ROS2 命名空间对应

### 与 ROS2 命名空间的关系
在实际部署中，机器人的帧名前缀通常与 ROS2 节点的命名空间一致：
```bash
ros2 run my_robot tf_publisher --ros-args -r __ns:=/robot1
```

### 常见场景
- 多机器人协同定位
- 机器人编队控制
- 多机器人 SLAM
- 物流仓库中的多 AGV 管理
