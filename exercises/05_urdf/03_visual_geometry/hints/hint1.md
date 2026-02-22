# 提示 1

测试首先检查每个 link 的 `<visual>` 中是否有 `<geometry>` 元素。

`<geometry>` 是 `<visual>` 的子元素，内部包含一种具体的几何形状。
三种基本形状的写法分别是：

```xml
<geometry>
  <box size="x y z"/>
</geometry>

<geometry>
  <cylinder radius="r" length="l"/>
</geometry>

<geometry>
  <sphere radius="r"/>
</geometry>
```

从 `base_link` 开始，它需要一个长方体。看看 TODO 1 的提示，
试着在 `<visual>` 中添加一个包含 `<box>` 的 `<geometry>` 元素。
