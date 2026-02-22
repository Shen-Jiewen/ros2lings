# 提示 1

测试检查每个 link 中是否有 `<collision>` 元素。

`<collision>` 的结构与 `<visual>` 很类似，只是不需要 `<material>`。
它定义了物理仿真引擎用于碰撞检测的几何形状。

最简单的做法是把 `<visual>` 中的 `<geometry>` 复制一份到 `<collision>` 中：

```xml
<collision>
  <geometry>
    <box size="1.0 0.5 0.3"/>
  </geometry>
</collision>
```

先从 `base_link` 开始，看看它的 `<visual>` 用了什么形状，
然后在 `<visual>` 后面添加一个相同形状的 `<collision>` 元素。
