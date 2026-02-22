# ROS2 参数系统 — 节点的运行时配置

## 概念

参数（Parameter）是 ROS2 节点的运行时配置机制。
每个节点可以声明自己需要的参数，并指定默认值和类型。
外部可以通过命令行、Launch 文件、YAML 配置文件来覆盖这些参数值。

参数类型包括：`bool`、`int`、`double`、`string`、以及它们的数组形式。

## 声明与获取参数

```cpp
// 声明参数（带默认值）
this->declare_parameter("max_speed", 10);           // 整数
this->declare_parameter("robot_name", "ros2bot");    // 字符串
this->declare_parameter("update_frequency", 30.0);   // 浮点数
this->declare_parameter("enable_debug", false);       // 布尔

// 获取参数值
int speed = this->get_parameter("max_speed").as_int();
std::string name = this->get_parameter("robot_name").as_string();
double freq = this->get_parameter("update_frequency").as_double();
bool debug = this->get_parameter("enable_debug").as_bool();
```

## 类型匹配规则

参数的默认值决定了它的类型，获取时必须使用对应的 `as_*()` 方法：

| 默认值类型   | 参数类型              | 获取方法        |
|-------------|----------------------|----------------|
| `10`        | PARAMETER_INTEGER    | `as_int()`     |
| `"hello"`   | PARAMETER_STRING     | `as_string()`  |
| `3.14`      | PARAMETER_DOUBLE     | `as_double()`  |
| `true`      | PARAMETER_BOOL       | `as_bool()`    |

## 关键点

- `declare_parameter()` 必须在使用参数之前调用
- 参数名在节点内必须唯一
- 类型不匹配会导致运行时异常（如用 `as_int()` 获取字符串类型参数）
- `get_parameter()` 中的参数名必须与 `declare_parameter()` 完全一致
- 参数也可以通过命令行设置：`ros2 run pkg node --ros-args -p max_speed:=20`
