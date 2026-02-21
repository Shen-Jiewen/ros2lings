# 节点生命周期基础 — Node 构造函数做了什么

## 概念

当你调用 `std::make_shared<rclcpp::Node>("name")` 时，
ROS2 在幕后做了很多初始化工作。理解这些对于调试很有帮助。

## Node 构造函数内部工作

```
std::make_shared<Node>("my_node")
        │
        ├── 设置节点名称和命名空间
        │
        ├── 获取/创建 ROS2 Context
        │
        ├── 创建 rcl_node_t（底层 C 结构）
        │
        ├── 初始化日志系统（/rosout 话题）
        │
        ├── 创建回调组（Callback Group）
        │
        └── 注册参数服务
```

## 节点默认属性

| 属性 | 默认值 | 获取方法 |
|------|--------|----------|
| 名称 | 构造参数 | `get_name()` |
| 命名空间 | "/" | `get_namespace()` |
| 完全限定名 | "/name" | `get_fully_qualified_name()` |
| 时钟类型 | ROS_TIME | `get_clock()->get_clock_type()` |

## 关键点

- `rclcpp::init()` 必须在创建任何节点之前调用
- 每个节点自动创建 `/rosout` 发布者用于日志
- `spin()` 是阻塞式事件循环；`spin_some()` 非阻塞
- `shutdown()` 销毁所有节点并释放 DDS 资源
- 节点名称在同一进程中应该唯一
