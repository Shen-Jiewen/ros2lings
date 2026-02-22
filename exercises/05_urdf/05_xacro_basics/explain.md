# Xacro 基础 -- 让 URDF 更强大

## 概念

Xacro（XML Macros）是 ROS 中用于增强 URDF 的宏语言。它在标准 XML 的基础上
添加了**属性**、**宏**、**条件**和**数学表达式**等功能，让你可以编写更简洁、
可维护的机器人模型文件。

Xacro 文件通常以 `.urdf.xacro` 或 `.xacro` 为扩展名，经过 xacro 处理器
转换后生成标准的 URDF 文件。

## 命名空间声明

每个 Xacro 文件必须在 `<robot>` 标签中声明 xacro 命名空间：

```xml
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="my_robot">
  <!-- Xacro 内容 -->
</robot>
```

**注意**: 命名空间 URL 必须是 `http://www.ros.org/wiki/xacro`，拼写错误会
导致 xacro 处理器无法识别 xacro 指令。

## xacro:property -- 属性

属性类似于变量，用于定义可复用的数值：

```xml
<!-- 定义属性 -->
<xacro:property name="wheel_radius" value="0.1" />
<xacro:property name="wheel_width" value="0.05" />

<!-- 使用属性 -->
<cylinder radius="${wheel_radius}" length="${wheel_width}" />
```

### 属性引用语法

使用 `${}` 语法引用属性值：

```xml
${property_name}              <!-- 引用属性 -->
${property_name * 2}          <!-- 数学运算 -->
${property_name + 0.1}        <!-- 加法 -->
```

**常见错误**: 使用 `$(property_name)` 而非 `${property_name}`。
`$()` 是 ROS launch 文件的替换语法，在 Xacro 中应使用 `${}`。

## xacro:macro -- 宏

宏允许你定义可复用的 XML 代码块：

```xml
<!-- 定义宏 -->
<xacro:macro name="wheel" params="name radius width">
  <link name="${name}_link">
    <visual>
      <geometry>
        <cylinder radius="${radius}" length="${width}" />
      </geometry>
    </visual>
  </link>
</xacro:macro>

<!-- 调用宏 -->
<xacro:wheel name="left_wheel" radius="0.1" width="0.05" />
<xacro:wheel name="right_wheel" radius="0.1" width="0.05" />
```

### 宏参数

- 在 `params` 属性中列出参数名，用空格分隔
- 调用时通过属性传递值
- **参数名必须完全匹配**：定义中是 `width`，调用时也必须是 `width`

### 带默认值的参数

```xml
<xacro:macro name="wheel" params="name radius width:=0.05">
  <!-- width 有默认值 0.05，调用时可以省略 -->
</xacro:macro>
```

### 块参数（Block Parameters）

```xml
<xacro:macro name="custom_link" params="name *content">
  <link name="${name}">
    <xacro:insert_block name="content" />
  </link>
</xacro:macro>
```

## 处理 Xacro 文件

将 Xacro 文件转换为标准 URDF：

```bash
# 使用 xacro 命令行工具
xacro robot.urdf.xacro > robot.urdf

# 在 launch 文件中使用
from launch_ros.descriptions import ParameterValue
from launch.substitutions import Command
robot_description = ParameterValue(
    Command(['xacro ', 'path/to/robot.urdf.xacro']),
    value_type=str
)
```

## Xacro 与 URDF 的关系

```
robot.urdf.xacro  -->  xacro 处理器  -->  robot.urdf（标准 URDF）
   (带宏/属性)                            (纯 XML，无宏语法)
```

Xacro 的好处：
- **减少重复**: 相似的 link/joint 用宏定义一次，调用多次
- **参数化**: 用属性定义尺寸，修改一处即可全局生效
- **可读性**: 更短、更清晰的模型文件

## 常见错误

1. **命名空间拼写错误** -- URL 中的 `xacro` 少写字母
2. **属性引用语法** -- 使用 `$()` 而非 `${}`
3. **宏参数名不匹配** -- 调用时的参数名与定义中的不一致
4. **忘记声明命名空间** -- 没有 `xmlns:xacro` 声明
