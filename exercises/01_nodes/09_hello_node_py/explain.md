# Python 节点（Node）— rclpy 基础

## 概念

rclpy 是 ROS2 的 Python 客户端库，提供与 rclcpp（C++）对应的 API。
每个 Python 节点同样是一个独立的功能模块，通过话题、服务、Action 通信。

## 生命周期

```
rclpy.init()                ← 初始化 ROS2 Python 上下文
       │
       ▼
node = Node('name')         ← 创建节点
       │
       ▼
  注册发布者/订阅者/服务/定时器
       │
       ▼
rclpy.spin(node)            ← 事件循环
       │
       ▼
node.destroy_node()         ← 销毁节点
       │
       ▼
rclpy.shutdown()            ← 清理资源
```

## 与 C++ 的对比

| C++ (rclcpp)                              | Python (rclpy)               |
|-------------------------------------------|------------------------------|
| `rclcpp::init(argc, argv)`               | `rclpy.init()`               |
| `std::make_shared<Node>("name")`          | `Node('name')`               |
| `rclcpp::spin(node)`                      | `rclpy.spin(node)`           |
| `rclcpp::spin_some(node)`                 | `rclpy.spin_once(node)`      |
| `rclcpp::shutdown()`                      | `rclpy.shutdown()`           |

## 关键点

- `rclpy.init()` 初始化全局 Context，必须在创建节点之前调用
- Python 节点不需要智能指针，直接用变量引用即可
- `spin()` 阻塞式事件循环；`spin_once()` 处理一次回调后返回
- `destroy_node()` 显式释放节点资源（Python 也有垃圾回收，但建议显式销毁）
