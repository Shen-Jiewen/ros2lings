# 提示 2

四个 TODO 的具体内容：

1. **TODO 1**: 创建广播器
   ```cpp
   tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
   ```

2. **TODO 2**: 创建 robot1 的两个变换
   - `world` -> `robot1/base_link`，平移 `(1.0, 0.0, 0.0)`
   - `robot1/base_link` -> `robot1/sensor`，平移 `(0.0, 0.0, 0.5)`

3. **TODO 3**: 创建 robot2 的两个变换，结构完全相同，只是：
   - 帧名前缀从 `robot1/` 改为 `robot2/`
   - base_link 的 x 平移从 `1.0` 改为 `-1.0`

4. **TODO 4**: 用 `sendTransform` 发布 `transforms` 向量
