# 提示 2

测试检查是否存在名为 `base_link` 的 link。

每个 `<link>` 标签都需要一个 `name` 属性。检查当前 URDF 中 `<link>` 的
`name` 属性值是什么 — 它是空的、拼写错误的、还是完全缺失的？

在 ROS2 中，机器人的根 link 通常命名为 `base_link`，这是一个广泛使用的约定。
