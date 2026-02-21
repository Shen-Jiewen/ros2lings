# 提示 3

完整的修复：将发布者的 QoS 改为与订阅者匹配：

```cpp
auto pub_qos = rclcpp::QoS(10);
pub_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
pub_qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
```

或者反过来，降低订阅者的要求也可以：
```cpp
auto sub_qos = rclcpp::QoS(10);
sub_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
sub_qos.durability(rclcpp::DurabilityPolicy::Volatile);
```
