# 提示 1

首先解决 TODO 1：创建 `tf2_ros::Buffer` 和 `tf2_ros::TransformListener`。

`Buffer` 负责存储和查询变换数据，创建时需要传入节点的时钟：
```cpp
tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
```

`TransformListener` 负责监听 TF 话题并把数据填入 Buffer：
```cpp
tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
```

注意 `TransformListener` 的构造函数接受的是 `Buffer` 的**引用**，不是指针。
