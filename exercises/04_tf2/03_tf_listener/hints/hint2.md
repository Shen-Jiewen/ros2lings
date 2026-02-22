# 提示 2

三个错误分别是：

1. `tf2_ros::Buffer` 的构造函数没有传入时钟参数。
   应改为 `tf2_ros::Buffer(this->get_clock())`。

2. `lookupTransform` 的参数顺序是 `(target_frame, source_frame, time)`。
   代码中 `"sensor_link"` 和 `"base_link"` 的位置反了。
   正确顺序应该是 `lookupTransform("base_link", "sensor_link", ...)`。

3. `lookupTransform` 调用没有被 `try-catch` 包裹。
   当变换不可用时会抛出 `tf2::TransformException`，
   如果不处理会导致节点崩溃。
