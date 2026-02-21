# 提示 2

组件节点的关键要素：
1. 类必须 `public` 继承 `rclcpp::Node`
2. 构造函数签名: `explicit TalkerComponent(const rclcpp::NodeOptions & options)`
3. 基类初始化: `: Node("talker_component", options)`
4. 使用 `this->create_publisher<>(...)` 创建发布者
5. 使用 `this->create_wall_timer(...)` 创建定时器
