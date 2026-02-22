# Xacro 参数化建模 -- 可配置的机械臂

## 概念

参数化建模是指通过**属性**（变量）控制模型的尺寸和形状，使得修改参数
就能自动调整整个模型。结合 Xacro 的**宏**功能，你可以用简洁的代码
描述重复出现的结构（如机械臂的多个关节段）。

## 设计思路

一个简单的双段机械臂可以分解为：

```
base_link（基座）
    |
    +-- segment_1_joint (revolute)
            |
            +-- segment_1（第一段臂）
                    |
                    +-- segment_2_joint (revolute)
                            |
                            +-- segment_2（第二段臂）
```

每段臂的结构完全相同（一个圆柱体 link + 一个 revolute joint），
只是名称和连接的父 link 不同。这正是宏的用武之地。

## 用 xacro:property 参数化尺寸

```xml
<xacro:property name="arm_length" value="0.5" />
<xacro:property name="arm_radius" value="0.05" />
```

以后只需修改这两个属性值，所有使用它们的地方都会自动更新。

## 用 xacro:macro 封装重复结构

```xml
<xacro:macro name="arm_segment" params="name parent length radius">
  <!-- link: 一个圆柱体 -->
  <link name="${name}">
    <visual>
      <origin xyz="0 0 ${length/2}" rpy="0 0 0" />
      <geometry>
        <cylinder radius="${radius}" length="${length}" />
      </geometry>
      <material name="arm_color">
        <color rgba="0.8 0.6 0.2 1.0" />
      </material>
    </visual>
  </link>

  <!-- joint: revolute 旋转关节 -->
  <joint name="${name}_joint" type="revolute">
    <parent link="${parent}" />
    <child link="${name}" />
    <origin xyz="0 0 ${length}" rpy="0 0 0" />
    <axis xyz="0 1 0" />
    <limit lower="-1.57" upper="1.57" effort="100.0" velocity="1.0" />
  </joint>
</xacro:macro>
```

### 关键设计细节

**visual origin 偏移**: 圆柱体的默认原点在其几何中心。为了让圆柱体
的底部对齐 joint 的位置，将 visual 的 origin 沿 z 轴偏移 `length/2`。

**joint origin**: joint 的 origin 决定了子 link 相对于父 link 的位置。
这里设为 `0 0 ${length}`，意味着子 link 的 joint 在父 link 的顶端。

注意：对于第一段臂（连接到 base_link），joint origin 的 z 偏移可以
设置为 base_link 的高度（如 0.1 的一半 = 0.05），或者简化为 `${length}`。
关键是确保模型在空间上连接正确。

## 调用宏构建机械臂

```xml
<!-- 第一段臂 -->
<xacro:arm_segment name="segment_1" parent="base_link"
  length="${arm_length}" radius="${arm_radius}" />

<!-- 第二段臂 -->
<xacro:arm_segment name="segment_2" parent="segment_1"
  length="${arm_length}" radius="${arm_radius}" />
```

每次调用宏会生成一个 link 和一个 joint，两次调用就构建了完整的双段臂。

## Xacro 中的数学表达式

Xacro 支持在 `${}` 中使用 Python 数学表达式：

```xml
${arm_length / 2}           <!-- 除法 -->
${arm_length * 2}           <!-- 乘法 -->
${arm_length + arm_radius}  <!-- 加法 -->
${pi / 4}                   <!-- 使用内置常量 pi -->
```

这在计算 origin 偏移、角度等场景中非常有用。

## 完整结构预览

```xml
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="param_arm">
  <!-- 属性 -->
  <xacro:property name="arm_length" value="0.5" />
  <xacro:property name="arm_radius" value="0.05" />

  <!-- 宏定义 -->
  <xacro:macro name="arm_segment" params="name parent length radius">
    <!-- ... link + joint ... -->
  </xacro:macro>

  <!-- 基座 -->
  <link name="base_link">...</link>

  <!-- 调用宏 -->
  <xacro:arm_segment name="segment_1" parent="base_link" ... />
  <xacro:arm_segment name="segment_2" parent="segment_1" ... />
</robot>
```

## 常见错误

1. **忘记定义 property** -- 在宏调用中使用了 `${arm_length}` 但没有定义属性
2. **宏参数遗漏** -- params 中少列了参数，或调用时少传了参数
3. **origin 偏移计算错误** -- 圆柱体中心需要偏移 `length/2`
4. **revolute joint 缺少 axis/limit** -- 这是必需元素
5. **parent link 名称不匹配** -- 第二段臂的 parent 应为第一段臂的 link 名称
