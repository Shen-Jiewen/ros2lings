# 提示 1

试着运行这段代码来探索 Node 的属性：
```cpp
auto node = std::make_shared<rclcpp::Node>("my_node");
std::cout << node->get_name() << std::endl;
std::cout << node->get_namespace() << std::endl;
```
