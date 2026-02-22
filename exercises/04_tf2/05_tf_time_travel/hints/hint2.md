# 提示 2

三个 BUG 分别是：

1. **时间戳**: `query_time` 被设置为未来 10 秒的时间。
   应该使用 `tf2::TimePointZero` 来获取最新可用的变换。

2. **超时**: `timeout` 设置为 `0.0` 秒，意味着完全不等待。
   如果变换还没被发布，`lookupTransform` 会立即失败。
   应该设置为一个合理的超时时间，比如 `1.0` 秒。

3. **异常类型**: `catch` 块捕获的是 `std::runtime_error`，
   但 TF2 的异常类型是 `tf2::TransformException`。
   需要包含 `<tf2/exceptions.h>` 并使用正确的类型。
