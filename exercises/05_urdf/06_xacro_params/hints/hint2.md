# 提示 2

第二步是定义 `xacro:macro`。

宏定义的语法：
```xml
<xacro:macro name="arm_segment" params="name parent length radius">
  <!-- 宏的内容 -->
</xacro:macro>
```

宏内部需要包含：

1. 一个 `<link>`，使用 `<cylinder>` 几何体：
```xml
<link name="${name}">
  <visual>
    <origin xyz="0 0 ${length/2}" rpy="0 0 0" />
    <geometry>
      <cylinder radius="${radius}" length="${length}" />
    </geometry>
  </visual>
</link>
```

2. 一个 `<joint>`，类型为 `revolute`：
```xml
<joint name="${name}_joint" type="revolute">
  <parent link="${parent}" />
  <child link="${name}" />
  <origin xyz="0 0 ${length}" />
  <axis xyz="0 1 0" />
  <limit lower="-1.57" upper="1.57" effort="100.0" velocity="1.0" />
</joint>
```

注意 `revolute` 类型的 joint 必须有 `<axis>` 和 `<limit>` 元素。
