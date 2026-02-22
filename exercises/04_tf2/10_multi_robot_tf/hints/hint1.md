# 提示 1

首先解决 TODO 1：创建 `StaticTransformBroadcaster`。

这一步和之前的练习完全相同：
```cpp
tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
```

然后在 TODO 2 中，你需要创建两个变换。关键是帧名要带 `robot1/` 前缀：
- `world` -> `robot1/base_link`
- `robot1/base_link` -> `robot1/sensor`

注意 `world` 帧是共享的，不需要前缀。
