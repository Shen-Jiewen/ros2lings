# 提示 3

完整的实现：

```cpp
// TODO 1: 创建广播器
tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

std::vector<geometry_msgs::msg::TransformStamped> transforms;

// TODO 2: robot1 的变换
geometry_msgs::msg::TransformStamped t1;
t1.header.stamp = this->now();
t1.header.frame_id = "world";
t1.child_frame_id = "robot1/base_link";
t1.transform.translation.x = 1.0;
t1.transform.translation.y = 0.0;
t1.transform.translation.z = 0.0;
t1.transform.rotation.w = 1.0;
transforms.push_back(t1);

geometry_msgs::msg::TransformStamped t2;
t2.header.stamp = this->now();
t2.header.frame_id = "robot1/base_link";
t2.child_frame_id = "robot1/sensor";
t2.transform.translation.x = 0.0;
t2.transform.translation.y = 0.0;
t2.transform.translation.z = 0.5;
t2.transform.rotation.w = 1.0;
transforms.push_back(t2);

// TODO 3: robot2 的变换
geometry_msgs::msg::TransformStamped t3;
t3.header.stamp = this->now();
t3.header.frame_id = "world";
t3.child_frame_id = "robot2/base_link";
t3.transform.translation.x = -1.0;
t3.transform.translation.y = 0.0;
t3.transform.translation.z = 0.0;
t3.transform.rotation.w = 1.0;
transforms.push_back(t3);

geometry_msgs::msg::TransformStamped t4;
t4.header.stamp = this->now();
t4.header.frame_id = "robot2/base_link";
t4.child_frame_id = "robot2/sensor";
t4.transform.translation.x = 0.0;
t4.transform.translation.y = 0.0;
t4.transform.translation.z = 0.5;
t4.transform.rotation.w = 1.0;
transforms.push_back(t4);

// TODO 4: 发布所有变换
tf_broadcaster_->sendTransform(transforms);
```
