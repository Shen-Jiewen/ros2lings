# 提示 3

完整的答案：

```python
answer_urdf_parameter = "robot_description"
answer_fixed_joint_topic = "/tf_static"
answer_movable_joint_publisher = "joint_state_publisher"
answer_joint_states_topic = "/joint_states"
answer_tf_frame_count = "N"
```

解释：
- `robot_description` 是 `robot_state_publisher` 接收 URDF 字符串的参数名
- `/tf_static` 用于 fixed joint 的变换（只发布一次，由 latched publisher 保持）
- `joint_state_publisher` 发布可动关节的状态到 `/joint_states`
- `robot_state_publisher` 订阅 `/joint_states`，计算并发布可动关节的 TF
- N 个 link 产生 N 个 TF 帧，每个 link 名就是帧的 frame_id
