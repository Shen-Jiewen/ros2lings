# 提示 1

这个 Launch 文件有 3 个问题：
1. 缺少了一个节点 — `LaunchDescription` 中应该有两个节点
2. 两个节点的名称不能相同 — 每个节点需要唯一的 `name`
3. talker 节点缺少了 `output='screen'` 参数
