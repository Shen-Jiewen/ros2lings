# 提示 1

编译错误通常是最先遇到的问题。如果编译器找不到 `tf2_ros::StaticTransformBroadcaster`，
说明缺少了对应的头文件。

检查文件顶部的 `#include` 部分，看看是否所有需要的头文件都已经包含了。
`StaticTransformBroadcaster` 类定义在 `tf2_ros/static_transform_broadcaster.h` 中。
