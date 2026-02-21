# 组件节点（Component Node）— 高效的进程内组合

## 概念

组件节点是 ROS2 推荐的节点组织方式。传统方式中，每个节点运行在独立进程中，
通过网络通信（即使在同一台机器上）。组件化后，多个节点可以加载到同一进程中，
实现零拷贝的进程内通信。

## 与普通节点的区别

```
普通节点:
  int main(...) {
    rclcpp::init(...);
    auto node = std::make_shared<MyNode>();
    rclcpp::spin(node);
  }

组件节点:
  class MyComponent : public rclcpp::Node {
    explicit MyComponent(const rclcpp::NodeOptions & options)
    : Node("name", options) { ... }
  };
  RCLCPP_COMPONENTS_REGISTER_NODE(MyComponent)
```

## 关键要求

1. **继承 `rclcpp::Node`**
2. **构造函数接受 `const rclcpp::NodeOptions &`**
3. **编译为共享库**（而非可执行文件）
4. **使用宏注册**: `RCLCPP_COMPONENTS_REGISTER_NODE(类名)`
5. **CMakeLists.txt** 中用 `add_library(SHARED)` 并调用 `rclcpp_components_register_nodes()`

## 加载方式

```bash
# 运行组件容器
ros2 run rclcpp_components component_container

# 动态加载组件
ros2 component load /ComponentManager \
  ros2lings_13_component_node TalkerComponent
```

## 优势

- 减少进程间通信开销
- 支持进程内零拷贝消息传递
- 运行时可动态加载/卸载
- 便于调试和性能分析
