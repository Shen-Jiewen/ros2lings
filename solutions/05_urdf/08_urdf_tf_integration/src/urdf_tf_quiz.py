#!/usr/bin/env python3

# ===== 问答区域 =====

# 问题 1: robot_state_publisher 节点从哪个参数读取 URDF？
answer_urdf_parameter = "robot_description"

# 问题 2: robot_state_publisher 将 fixed joint 的变换发布到哪个话题？
answer_fixed_joint_topic = "/tf_static"

# 问题 3: 哪个节点负责发布 revolute/prismatic 等可动关节的状态？
answer_movable_joint_publisher = "joint_state_publisher"

# 问题 4: joint_state_publisher 发布消息到哪个话题？
answer_joint_states_topic = "/joint_states"

# 问题 5: 一个有 N 个 link 和 N-1 个 joint 的 URDF 会产生多少个 TF 帧？
answer_tf_frame_count = "N"

# ===== 结束 =====

if __name__ == '__main__':
    print(f'answer_urdf_parameter = {answer_urdf_parameter}')
    print(f'answer_fixed_joint_topic = {answer_fixed_joint_topic}')
    print(f'answer_movable_joint_publisher = {answer_movable_joint_publisher}')
    print(f'answer_joint_states_topic = {answer_joint_states_topic}')
    print(f'answer_tf_frame_count = {answer_tf_frame_count}')
