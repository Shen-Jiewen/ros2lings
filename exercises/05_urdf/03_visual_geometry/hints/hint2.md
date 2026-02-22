# 提示 2

测试还检查每个 `<visual>` 中是否有 `<material>` 元素。

`<material>` 需要一个 `name` 属性，内部用 `<color>` 定义 RGBA 颜色值：

```xml
<material name="blue">
  <color rgba="0 0 1 1"/>
</material>
```

`rgba` 的四个值分别是 红(R)、绿(G)、蓝(B)、透明度(A)，范围 0.0~1.0。

- 蓝色：`rgba="0 0 1 1"`
- 绿色：`rgba="0 1 0 1"`
- 红色：`rgba="1 0 0 1"`

为每个 link 的 `<visual>` 添加对应颜色的 `<material>` 元素。
