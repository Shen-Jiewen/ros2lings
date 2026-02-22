# TF2 时间戳查询 — TimePointZero 与异常处理

## 概念

TF2 的 `lookupTransform` 不仅可以查询两个帧之间的空间关系，
还可以查询**特定时刻**的变换。这意味着 TF2 缓冲区（Buffer）中保存了
变换的**历史记录**，支持"时间旅行"式的查询。

## lookupTransform 完整签名

```cpp
geometry_msgs::msg::TransformStamped lookupTransform(
  const std::string & target_frame,  // 目标帧
  const std::string & source_frame,  // 源帧
  const tf2::TimePoint & time,       // 查询时间
  const tf2::Duration & timeout      // 等待超时
);
```

## tf2::TimePointZero

`tf2::TimePointZero` 是一个特殊的时间值，表示"获取最新可用的变换"。

```cpp
// 获取最新可用的变换（推荐）
auto t = buffer->lookupTransform("world", "robot", tf2::TimePointZero);

// 查询特定时刻的变换（需要 buffer 中有该时刻的数据）
auto t = buffer->lookupTransform("world", "robot", some_specific_time);
```

**为什么推荐使用 `TimePointZero`?**
- 不需要知道变换的确切发布时间
- 不会因为时钟不同步而查询失败
- 始终返回最新的可用数据

## timeout 参数

timeout 参数指定了 `lookupTransform` 愿意**等待**多长时间。

```cpp
// 不等待，立即返回或抛异常（timeout = 0）
auto t = buffer->lookupTransform("world", "robot",
  tf2::TimePointZero, rclcpp::Duration(0, 0).to_chrono<std::chrono::nanoseconds>());

// 等待最多 1 秒（推荐）
auto t = buffer->lookupTransform("world", "robot",
  tf2::TimePointZero,
  rclcpp::Duration::from_seconds(1.0).to_chrono<std::chrono::nanoseconds>());
```

**为什么 timeout > 0 很重要?**
- 变换数据通过话题异步传输，可能还没到达
- 给 TransformListener 时间来接收和处理数据
- 避免启动顺序导致的"变换不可用"问题

## 异常处理

`lookupTransform` 可能抛出以下异常（都继承自 `tf2::TransformException`）：

| 异常类型 | 触发条件 |
|---------|---------|
| `tf2::LookupException` | 请求的帧不存在 |
| `tf2::ConnectivityException` | 两帧之间没有连接路径 |
| `tf2::ExtrapolationException` | 请求的时间超出缓冲区范围 |

```cpp
#include <tf2/exceptions.h>

try {
  auto t = buffer->lookupTransform("world", "robot", tf2::TimePointZero);
} catch (const tf2::TransformException & ex) {
  // 捕获所有 TF2 异常
  RCLCPP_WARN(this->get_logger(), "%s", ex.what());
}
```

**注意**: 应该捕获 `tf2::TransformException`，而不是 `std::runtime_error` 或
`std::exception`。虽然后者也能编译通过，但语义不够精确，可能掩盖其他非 TF2 的错误。

## 常见错误

### 使用"未来"时间查询
```cpp
// 错误：查询 10 秒后的变换，buffer 中不可能有未来的数据
auto future = std::chrono::system_clock::now() + std::chrono::seconds(10);
auto t = buffer->lookupTransform("world", "robot", tf2::TimePoint(future));

// 正确：使用 TimePointZero
auto t = buffer->lookupTransform("world", "robot", tf2::TimePointZero);
```

### timeout 为零
```cpp
// 不好：不等待，变换没准备好就立即失败
buffer->lookupTransform("world", "robot", tf2::TimePointZero,
  rclcpp::Duration(0, 0).to_chrono<std::chrono::nanoseconds>());

// 好：等待合理时间
buffer->lookupTransform("world", "robot", tf2::TimePointZero,
  rclcpp::Duration::from_seconds(1.0).to_chrono<std::chrono::nanoseconds>());
```
