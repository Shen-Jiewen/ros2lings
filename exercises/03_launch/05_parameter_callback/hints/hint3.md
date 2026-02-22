# 提示 3

完整的修复：

1. 在构造函数中取消注释回调注册代码：
   ```cpp
   callback_handle_ = this->add_on_set_parameters_callback(
     std::bind(&DynamicParamNode::param_callback, this, std::placeholders::_1));
   ```

2. 把 `result.successful = false` 改为 `result.successful = true`

3. 在回调的 `if (param.get_name() == "speed")` 分支中添加：
   ```cpp
   speed_ = param.as_int();
   ```
