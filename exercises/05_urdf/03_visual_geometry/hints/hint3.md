# 提示 3

以下是所有 TODO 的完整解答。每个 link 的 `<visual>` 中需要添加
`<geometry>` 和 `<material>` 两个子元素：

**base_link**（长方体 + 蓝色）：
```xml
<geometry>
  <box size="1.0 0.5 0.1"/>
</geometry>
<material name="blue">
  <color rgba="0 0 1 1"/>
</material>
```

**body**（圆柱体 + 绿色）：
```xml
<geometry>
  <cylinder radius="0.2" length="0.6"/>
</geometry>
<material name="green">
  <color rgba="0 1 0 1"/>
</material>
```

**head**（球体 + 红色）：
```xml
<geometry>
  <sphere radius="0.15"/>
</geometry>
<material name="red">
  <color rgba="1 0 0 1"/>
</material>
```

完成后记得删除文件顶部的 `<!-- I AM NOT DONE -->` 注释。
