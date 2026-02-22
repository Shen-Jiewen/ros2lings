# 提示 3

完整的修复：

1. 取消注释头文件包含：
   ```cpp
   #include <tf2_ros/static_transform_broadcaster.h>
   ```

2. 交换 `frame_id` 和 `child_frame_id` 的值：
   ```cpp
   t.header.frame_id = "world";
   t.child_frame_id = "base_link";
   ```

3. 将四元数的 `w` 分量从 `0.0` 改为 `1.0`：
   ```cpp
   t.transform.rotation.w = 1.0;
   ```
