# 服务内省 — 用 CLI 工具调试 ROS2 服务

## 概念

ROS2 提供了一套命令行工具来查看和调试正在运行的服务。
这些工具对于理解系统中有哪些服务、如何调用它们非常有用。

## 常用命令

```bash
# 列出所有活跃服务
ros2 service list

# 查看服务类型
ros2 service type /add_two_ints

# 查找特定类型的服务
ros2 service find ros2lings_interfaces/srv/AddTwoInts

# 从命令行调用服务
ros2 service call /add_two_ints ros2lings_interfaces/srv/AddTwoInts "{a: 1, b: 2}"

# 查看服务接口定义
ros2 interface show ros2lings_interfaces/srv/AddTwoInts
```

## Service vs Topic 对比

| 特性 | Service | Topic |
|------|---------|-------|
| 通信模式 | 请求-响应 (request_response) | 发布-订阅 (publish_subscribe) |
| 同步性 | 同步（等待响应） | 异步（发后不管） |
| 保证交付 | 是（服务端运行时） | 否（取决于 QoS） |
| 关系 | 一对一 | 一对多/多对多 |
| 适用场景 | 偶尔的请求（如参数查询） | 持续的数据流（如传感器） |

## 关键点

- `ros2 service list` 列出所有活跃服务（包括系统自带的）
- 每个节点自动创建一些内置服务（如参数相关的服务）
- `ros2 service call` 可以从命令行直接调用服务进行测试
- Service 保证在服务端正常运行时会返回响应
- 使用 `ros2 interface show` 查看服务的请求/响应结构
