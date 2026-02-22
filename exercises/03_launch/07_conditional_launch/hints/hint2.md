# 提示 2

- `IfCondition(LaunchConfiguration('use_sim'))` — 当 `use_sim='true'` 时启动
- `UnlessCondition(LaunchConfiguration('use_sim'))` — 当 `use_sim='false'` 时启动

两个节点都需要 `condition` 参数，且都需要放入 `LaunchDescription` 列表中。
`DeclareLaunchArgument` 也要放入列表。
