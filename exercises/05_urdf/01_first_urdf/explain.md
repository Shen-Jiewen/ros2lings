# URDF 基础 — 你的第一个机器人模型

## 概念

URDF（Unified Robot Description Format）是一种基于 XML 的格式，
用于描述机器人的结构、外观和物理属性。ROS2 中几乎所有的机器人模型
都使用 URDF 来定义。

## URDF 文件的基本结构

```xml
<?xml version="1.0"?>
<robot name="my_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="1.0 0.5 0.5"/>
      </geometry>
    </visual>
  </link>
</robot>
```

## 核心元素

### `<robot>` — 根元素

- 每个 URDF 文件必须有且只有一个 `<robot>` 根元素
- **必须**包含 `name` 属性，用来给机器人命名
- 所有的 link 和 joint 都定义在 `<robot>` 内部

```xml
<robot name="my_robot">
  <!-- link 和 joint 定义在这里 -->
</robot>
```

### `<link>` — 连杆

Link（连杆）是机器人的刚性部件。每个 link **必须**有 `name` 属性。

```xml
<link name="base_link">
  <!-- 可选：visual, collision, inertial -->
</link>
```

一个 link 可以包含：
- `<visual>` — 可视化外观（在 RViz 中显示）
- `<collision>` — 碰撞检测形状（用于物理仿真）
- `<inertial>` — 惯性属性（质量、转动惯量）

### `<visual>` — 可视化

```xml
<visual>
  <geometry>
    <box size="1.0 0.5 0.5"/>   <!-- 长方体：长 宽 高 -->
  </geometry>
  <material name="blue">
    <color rgba="0.0 0.0 0.8 1.0"/>  <!-- 红 绿 蓝 透明度 -->
  </material>
</visual>
```

### `<geometry>` — 几何形状

URDF 支持四种基本几何形状：

| 形状 | XML | 参数 |
|------|-----|------|
| 长方体 | `<box size="x y z"/>` | 三个维度（米） |
| 圆柱体 | `<cylinder radius="r" length="l"/>` | 半径和长度（米） |
| 球体 | `<sphere radius="r"/>` | 半径（米） |
| 网格 | `<mesh filename="..."/>` | 3D 模型文件路径 |

**注意**: `<box>` 的 `size` 属性必须包含 3 个用空格分隔的数值，
分别对应 x（长）、y（宽）、z（高）方向的尺寸。

## base_link 的约定

在 ROS2 中，`base_link` 是一个约定俗成的名称，表示机器人的根坐标帧。
所有其他 link 都通过 joint 与 `base_link`（直接或间接）相连。

## robot_state_publisher

`robot_state_publisher` 节点读取 URDF 模型并将机器人的状态发布到 TF2 树中，
让 RViz 等可视化工具可以显示机器人模型。

```bash
# 加载 URDF 并启动 robot_state_publisher
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat robot.urdf)"
```

## 常见错误

1. `<robot>` 缺少 `name` 属性 — 解析器会报错
2. `<link>` 的 `name` 为空或缺失 — 无法被 joint 引用
3. `<box size="1.0"/>` — 缺少维度，必须是三个值
4. XML 标签未正确关闭 — URDF 是严格的 XML 格式
