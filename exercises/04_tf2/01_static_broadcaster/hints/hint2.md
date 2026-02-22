# 提示 2

三个错误分别是：

1. 缺少头文件 `#include <tf2_ros/static_transform_broadcaster.h>`，
   被注释掉了，需要取消注释。

2. `header.frame_id` 和 `child_frame_id` 的值被设置反了。
   `header.frame_id` 应该是父帧 `"world"`，
   `child_frame_id` 应该是子帧 `"base_link"`。

3. 四元数的 `w` 分量被设为 `0.0`，这不是一个有效的旋转。
   单位四元数（无旋转）的 `w` 应为 `1.0`。
