# 提示 2

- 头文件：`#include <ros2lings_19_custom_srv/srv/compute_area.hpp>`
- C++ 类型：`ros2lings_19_custom_srv::srv::ComputeArea`
- Request 字段：`request->width`, `request->height`（类型 float64 对应 C++ double）
- Response 字段：`response->area`
- 面积计算：`response->area = request->width * request->height;`
