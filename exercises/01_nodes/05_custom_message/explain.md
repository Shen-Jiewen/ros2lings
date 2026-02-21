# 自定义消息（Custom Message）— 定义你自己的数据结构

## 概念

ROS2 使用 .msg 文件定义消息格式。编译时，rosidl 会自动生成
C++、Python 等语言的代码，让你可以在代码中直接使用。

## .msg 文件格式

```
# msg/SensorData.msg
float64 temperature    # 温度
float64 humidity       # 湿度
string sensor_id       # 传感器ID
```

## 构建配置

```
# CMakeLists.txt 中需要:
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SensorData.msg"
)

# package.xml 中需要:
<buildtool_depend>rosidl_default_generators</buildtool_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

## 头文件命名规则

- 包名: `ros2lings_05_custom_message`
- 消息名: `SensorData`
- 头文件: `ros2lings_05_custom_message/msg/sensor_data.hpp`
- C++ 类型: `ros2lings_05_custom_message::msg::SensorData`

## 关键点

- .msg 文件放在包的 `msg/` 目录下
- 字段类型遵循 ROS2 IDL 规范（float64、string、int32 等）
- 消息名用 CamelCase，生成的头文件用 snake_case
- 同一个包中使用自定义消息需要 `rosidl_get_typesupport_target`
