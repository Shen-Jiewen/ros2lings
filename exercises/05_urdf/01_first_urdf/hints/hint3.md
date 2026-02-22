# 提示 3

`<box>` 的 `size` 属性需要**三个**用空格分隔的数值，分别表示 x、y、z 方向的尺寸。

错误写法：
```xml
<box size="1.0"/>          <!-- 只有 1 个值 -->
<box size="1.0 0.5"/>      <!-- 只有 2 个值 -->
```

正确写法：
```xml
<box size="1.0 0.5 0.5"/>  <!-- 3 个值：长 宽 高 -->
```

完整的修复后的 URDF 应该类似于：
```xml
<robot name="my_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="1.0 0.5 0.5"/>
      </geometry>
    </visual>
  </link>
</robot>
```
