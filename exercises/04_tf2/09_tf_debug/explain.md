# TF2 调试与分析 — TF Debug

## 概念

调试 TF 树是机器人开发中非常常见的任务。TF2 提供了多个工具和话题来帮助
开发者理解和排查坐标变换问题。

## TF2 调试工具

### view_frames

`view_frames` 是 `tf2_tools` 包中最常用的调试工具，
它会监听 TF 话题并生成一个 TF 树的 PDF 可视化图：

```bash
ros2 run tf2_tools view_frames
```

生成的 PDF 文件显示：
- 所有坐标帧及其父子关系
- 变换的发布频率
- 最近一次更新的时间

### tf2_echo

查看两个帧之间的实时变换：

```bash
ros2 run tf2_ros tf2_echo map base_link
```

### tf2_monitor

监控 TF 树的整体状态：

```bash
ros2 run tf2_ros tf2_monitor
```

## TF2 话题

TF2 使用两个标准话题来传输变换数据：

| 话题 | 类型 | 用途 |
|------|------|------|
| `/tf` | `tf2_msgs/TFMessage` | 动态变换（需持续发布） |
| `/tf_static` | `tf2_msgs/TFMessage` | 静态变换（只需发布一次，使用 transient local QoS） |

可以用以下命令查看：
```bash
ros2 topic echo /tf
ros2 topic echo /tf_static
```

## lookupTransform 的方向

```cpp
auto t = buffer->lookupTransform(target_frame, source_frame, time);
```

返回的变换方向是**从 source 到 target**（source_to_target）。
也就是说，这个变换可以将 source_frame 中的数据转换到 target_frame 中。

直觉理解：
- 你想要数据在 `target_frame` 中的表示
- 数据当前在 `source_frame` 中
- 所以需要一个"从 source 到 target"的变换

## TF 树的规则

### 树结构
TF2 中的帧关系必须形成一棵**树**（tree），而不是图（graph）：
- 每个子帧**只能有一个父帧**
- 不允许出现环（cycle）

### 多父节点错误
如果两个不同的发布者试图为同一个子帧设置不同的父帧，
TF2 会产生错误。这是因为树结构不允许一个节点有多个父节点。

例如，如果同时发布：
- `A -> C`（A 是 C 的父帧）
- `B -> C`（B 也是 C 的父帧）

TF2 会报错，因为 C 不能同时有两个父帧。

### 常见调试场景
- **帧不存在**: 检查发布者是否正在运行
- **变换超时**: 动态变换可能过期，检查发布频率
- **帧链断裂**: 中间帧缺失，使用 `view_frames` 可视化检查
- **多父节点冲突**: 检查是否有多个节点发布同一子帧的变换
