# 提示 2

三个错误分别是：

1. `this->declare_parameter("max_speed", "fast")` — `"fast"` 是字符串，
   但后面用 `as_int()` 来获取，类型不匹配。应改为整数，如 `10`。

2. `this->get_parameter("robot_nam")` — 参数名少了一个字母 `e`，
   应改为 `"robot_name"`。

3. `update_frequency` 参数的声明和获取都被注释掉了，需要取消注释。
