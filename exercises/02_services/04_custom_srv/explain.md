# 自定义服务（Custom Service）— 定义你自己的请求/响应接口

## 概念

与自定义消息（.msg）类似，ROS2 允许你定义自定义的 .srv 文件来描述
服务的请求和响应格式。编译时，rosidl 自动生成对应语言的代码。

## .srv 文件格式

```
# srv/ComputeArea.srv
float64 width      # 请求部分
float64 height
---                 # 分隔符
float64 area        # 响应部分
```

用 `---` 分隔请求和响应两部分。上面是请求字段，下面是响应字段。

## 构建配置

### CMakeLists.txt

```cmake
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ComputeArea.srv"
)

# 如果同一个包中要使用自定义服务：
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(custom_srv "${cpp_typesupport_target}")
```

### package.xml

```xml
<buildtool_depend>rosidl_default_generators</buildtool_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

## 头文件命名规则

| 元素 | 值 |
|------|-----|
| 包名 | `ros2lings_19_custom_srv` |
| 服务名 | `ComputeArea` |
| 头文件 | `ros2lings_19_custom_srv/srv/compute_area.hpp` |
| C++ 类型 | `ros2lings_19_custom_srv::srv::ComputeArea` |
| Request | `ros2lings_19_custom_srv::srv::ComputeArea::Request` |
| Response | `ros2lings_19_custom_srv::srv::ComputeArea::Response` |

规则：CamelCase 转为 snake_case（`ComputeArea` -> `compute_area`）。

## IDL 类型对应

| ROS2 IDL | C++ | Python |
|----------|-----|--------|
| float64 | double | float |
| float32 | float | float |
| int64 | int64_t | int |
| string | std::string | str |
| bool | bool | bool |

## 关键点

- .srv 文件放在包的 `srv/` 目录下
- 服务名用 CamelCase，头文件用 snake_case
- 同一个包中使用自定义服务需要 `rosidl_get_typesupport_target`
- rosidl 会同时生成 C++ 和 Python 代码
- 修改 .srv 文件后需要重新构建
