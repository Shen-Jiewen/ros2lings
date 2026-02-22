# 提示 1

URDF 与 TF2 集成涉及两个关键节点和它们的分工：

- `robot_state_publisher` — 读取 URDF 模型，发布坐标变换
- `joint_state_publisher` — 发布各关节的状态（角度、位置）

思考以下问题：
- `robot_state_publisher` 从哪里获取 URDF？是文件路径还是参数？
- 固定不动的关节（fixed joint）和可以动的关节（revolute/prismatic），
  它们的变换分别发布到哪里？
- 每个 link 在 TF 树中对应什么？
