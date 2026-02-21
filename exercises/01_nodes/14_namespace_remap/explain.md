# 命名空间与重映射 — 灵活的名称管理

## 命名空间（Namespace）

命名空间用于逻辑隔离节点和话题，特别适合多机器人场景：

```
/robot1/data_publisher  ← robot1 的传感器节点
/robot1/sensor_data     ← robot1 的传感器话题

/robot2/data_publisher  ← robot2 的传感器节点
/robot2/sensor_data     ← robot2 的传感器话题
```

### 设置方式

```cpp
// 代码中设置
auto node = std::make_shared<rclcpp::Node>("name", "namespace");

// 命令行设置
ros2 run pkg node --ros-args -r __ns:=/robot1
```

## 话题重映射（Remapping）

重映射允许在不修改代码的情况下改变话题名：

```bash
# 命令行重映射
ros2 run pkg node --ros-args -r raw_data:=sensor_data

# 多个重映射
ros2 run pkg node --ros-args \
  -r __ns:=/robot1 \
  -r raw_data:=sensor_data
```

### 代码中重映射

```cpp
rclcpp::NodeOptions options;
options.arguments({
  "--ros-args", "-r", "raw_data:=sensor_data"
});
auto node = std::make_shared<MyNode>(options);
```

## 话题名称解析规则

| 话题名     | 命名空间  | 完全限定名           |
|-----------|----------|---------------------|
| `chatter` | `/ns`    | `/ns/chatter`       |
| `/chatter`| `/ns`    | `/chatter` (绝对路径) |
| `~chatter`| `/ns`    | `/ns/node/chatter`  |
