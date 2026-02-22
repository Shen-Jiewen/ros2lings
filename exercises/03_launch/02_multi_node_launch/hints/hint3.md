# 提示 3

完整的修复：
1. 把 `LaunchDescription([talker])` 改为 `LaunchDescription([talker, listener])`
2. 把 listener 的 `name='talker'` 改为 `name='listener'`
3. 给 talker 节点添加 `output='screen'`
