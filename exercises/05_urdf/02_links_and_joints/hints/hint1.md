# 提示 1

测试检查所有 joint 的 `type` 属性是否有效。有效的关节类型包括：
`revolute`、`continuous`、`prismatic`、`fixed`、`floating`、`planar`。

如果你看到类似 "不是有效的关节类型" 的错误，仔细检查每个 `<joint>` 标签的
`type` 属性，看看是否有拼写错误。

英文拼写要格外小心，比如 "revolute" 这个词很容易漏掉最后的 "e"。
