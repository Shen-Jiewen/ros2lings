# 提示 3

`revolute` 类型的 joint 必须包含 `<axis>` 和 `<limit>` 元素。

`<axis>` 指定旋转轴方向，`<limit>` 指定运动范围和力矩限制。

在 `base_to_upper` joint 中添加以下两个元素：

```xml
<axis xyz="0 1 0"/>
<limit lower="-1.57" upper="1.57" effort="100.0" velocity="1.0"/>
```

修复后完整的 joint 应该类似于：
```xml
<joint name="base_to_upper" type="revolute">
  <parent link="base_link"/>
  <child link="upper_arm"/>
  <origin xyz="0 0 0.025" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-1.57" upper="1.57" effort="100.0" velocity="1.0"/>
</joint>

<joint name="upper_to_lower" type="fixed">
  <parent link="upper_arm"/>
  <child link="lower_arm"/>
  <origin xyz="0 0 0.5" rpy="0 0 0"/>
</joint>
```
