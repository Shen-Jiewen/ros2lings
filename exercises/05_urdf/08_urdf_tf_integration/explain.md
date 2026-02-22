# URDF 与 TF2 集成 — 从模型到坐标变换

## 概念

URDF 定义了机器人的结构（link 和 joint），而 TF2 系统负责在运行时维护各坐标帧
之间的变换关系。`robot_state_publisher` 和 `joint_state_publisher` 是将这两个
系统连接起来的桥梁。

## 整体架构

```
URDF 文件
    |
    v
robot_state_publisher ──┬──> /tf_static  (fixed joint 变换)
    ^                   └──> /tf         (可动 joint 变换)
    |
/joint_states
    ^
    |
joint_state_publisher  (发布关节角度/位置)
```

## robot_state_publisher

### 工作原理

1. **读取 URDF** — 通过 `robot_description` 参数接收 URDF 字符串
2. **解析模型** — 提取所有 link 和 joint 的定义
3. **发布固定变换** — 将 `fixed` 类型的 joint 变换发布到 `/tf_static`（只发布一次）
4. **订阅关节状态** — 监听 `/joint_states` 话题
5. **计算并发布动态变换** — 根据收到的关节状态，计算可动 joint 的变换并发布到 `/tf`

### 参数

| 参数名 | 类型 | 说明 |
|--------|------|------|
| `robot_description` | string | URDF 模型的完整 XML 字符串 |

### 话题

| 话题 | 方向 | 消息类型 | 说明 |
|------|------|----------|------|
| `/joint_states` | 订阅 | sensor_msgs/JointState | 关节状态输入 |
| `/tf` | 发布 | tf2_msgs/TFMessage | 可动关节的变换 |
| `/tf_static` | 发布 | tf2_msgs/TFMessage | 固定关节的变换 |
| `/robot_description` | 发布 | std_msgs/String | URDF 模型描述 |

## joint_state_publisher

### 工作原理

`joint_state_publisher` 负责发布机器人各个可动关节的状态（角度、速度等）。

- **输入**: 从 URDF 中解析出所有非固定关节
- **输出**: 将关节状态发布到 `/joint_states` 话题
- **消息类型**: `sensor_msgs/msg/JointState`

### 两种模式

1. **joint_state_publisher** — 发布所有关节的默认值（通常为 0）
2. **joint_state_publisher_gui** — 提供滑动条 GUI，可以手动调节关节角度

## TF 帧与 URDF 的关系

### 每个 link 对应一个 TF 帧

一个有 N 个 link 和 N-1 个 joint 的 URDF（树形结构）会产生 **N 个 TF 帧**。
每个 link 的 `name` 就是对应 TF 帧的 `frame_id`。

### 示例

```xml
<robot name="simple_robot">
  <link name="base_link"/>       <!-- TF 帧: base_link -->
  <link name="arm_link"/>        <!-- TF 帧: arm_link -->
  <joint name="base_to_arm" type="revolute">
    <parent link="base_link"/>
    <child link="arm_link"/>
  </joint>
</robot>
```

这个 URDF 有 2 个 link，产生 2 个 TF 帧：`base_link` 和 `arm_link`。

### fixed vs 可动 joint

| Joint 类型 | TF 发布话题 | 更新频率 |
|------------|-------------|----------|
| `fixed` | `/tf_static` | 只发布一次 |
| `revolute` | `/tf` | 随 `/joint_states` 更新 |
| `prismatic` | `/tf` | 随 `/joint_states` 更新 |
| `continuous` | `/tf` | 随 `/joint_states` 更新 |

## 完整工作流

1. 编写 URDF 文件，定义 link 和 joint
2. 在 Launch 文件中启动 `robot_state_publisher`，传入 URDF
3. 启动 `joint_state_publisher`（或 GUI 版本）
4. `joint_state_publisher` 发布关节状态到 `/joint_states`
5. `robot_state_publisher` 收到关节状态后，计算并发布 TF 变换
6. RViz 等工具读取 TF 变换，显示机器人模型

## 常见问题

1. **TF 树不完整** — 通常是因为缺少 `joint_state_publisher`，可动关节没有状态数据
2. **模型不显示** — 检查 `robot_description` 参数是否正确传递
3. **关节不动** — 确认 `joint_state_publisher` 正在运行，且发布了正确的关节名称
