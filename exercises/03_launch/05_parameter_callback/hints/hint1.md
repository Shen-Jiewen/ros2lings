# 提示 1

`add_on_set_parameters_callback` 用来注册参数变化的回调。
注意它返回一个句柄（SharedPtr），你必须把这个句柄保存为成员变量，
否则回调会被立刻销毁。

回调函数的返回值类型是 `rcl_interfaces::msg::SetParametersResult`。
