# 提示 3

测试检查宏调用时的参数名是否与宏定义中的参数名一致。

宏定义中的参数列表为：
```xml
<xacro:macro name="box_link" params="name width length height color_name r g b">
```

调用时，每个参数名必须**精确匹配**。检查第二个宏调用（创建 top_link 的那个），
看看是否有参数名拼写错误。

修复后完整的 Xacro 文件应该类似于：
```xml
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="xacro_robot">
  <xacro:property name="base_width" value="0.5" />
  <xacro:property name="base_length" value="1.0" />
  <xacro:property name="base_height" value="0.2" />

  <xacro:macro name="box_link" params="name width length height color_name r g b">
    <link name="${name}">
      <visual>
        <geometry>
          <box size="${width} ${length} ${height}" />
        </geometry>
        <material name="${color_name}">
          <color rgba="${r} ${g} ${b} 1.0" />
        </material>
      </visual>
    </link>
  </xacro:macro>

  <xacro:box_link name="base_link"
    width="${base_width}" length="${base_length}" height="${base_height}"
    color_name="blue" r="0" g="0" b="1" />

  <xacro:box_link name="top_link"
    width="0.3" length="0.3" height="0.1"
    color_name="red" r="1" g="0" b="0" />

  <joint name="base_to_top" type="fixed">
    <parent link="base_link" />
    <child link="top_link" />
    <origin xyz="0 0 ${base_height}" />
  </joint>
</robot>
```
