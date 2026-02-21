# 自定义 Action — 定义你自己的 Action 接口

## 概念

ROS2 允许你定义自己的 Action 类型，就像自定义消息和服务一样。
一个 `.action` 文件由三部分组成，用 `---` 分隔:

```
# Goal (目标请求)
int32 target_number
---
# Result (最终结果)
int32 final_count
---
# Feedback (中间反馈)
int32 current_count
```

## 文件结构

```
package_name/
├── action/
│   └── Countdown.action    ← Action 定义文件
├── CMakeLists.txt           ← 需要 rosidl_generate_interfaces
├── package.xml              ← 需要 rosidl 依赖
└── src/
    └── custom_action.cpp
```

## CMakeLists.txt 关键配置

```cmake
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Countdown.action"
)

# 链接生成的类型支持
rosidl_get_typesupport_target(typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(custom_action "${typesupport_target}")
```

## package.xml 关键依赖

```xml
<buildtool_depend>rosidl_default_generators</buildtool_depend>
<depend>rosidl_default_runtime</depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

## 生成的头文件路径

`Countdown.action` 生成的 C++ 头文件:
```cpp
#include "包名/action/countdown.hpp"
```

注意: 文件名自动转为小写蛇形命名。

## 关键点

- `.action` 文件放在 `action/` 目录下
- 文件名首字母大写 (CamelCase)
- 生成的头文件名是小写蛇形 (snake_case)
- 需要 `rosidl_default_generators` 来编译
- 同一个包中的代码需要用 `rosidl_get_typesupport_target` 链接
