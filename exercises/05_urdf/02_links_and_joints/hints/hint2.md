# 提示 2

测试检查每个 joint 的 `<parent>` 和 `<child>` 引用的 link 名称是否在
已定义的 link 列表中。

这个机器人定义了三个 link：`base_link`、`upper_arm`、`lower_arm`。
检查每个 joint 的 `<parent link="..."/>` 和 `<child link="..."/>`，
确保引用的名称与上面的 link 名称**完全一致**。

常见错误是使用了简写形式，比如 `"upper"` 而不是 `"upper_arm"`。
