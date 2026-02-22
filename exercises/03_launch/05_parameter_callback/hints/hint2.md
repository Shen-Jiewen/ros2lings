# 提示 2

三个错误分别是：

1. 构造函数中没有调用 `add_on_set_parameters_callback` 来注册回调。
   需要取消注释并用 `std::bind` 绑定 `param_callback` 方法。

2. `param_callback` 中的 `result.successful` 被设为 `false`，
   这会拒绝所有参数修改。应改为 `true`。

3. 回调中没有更新 `speed_` 成员变量，
   需要在检测到 `"speed"` 参数时赋值：`speed_ = param.as_int()`。
