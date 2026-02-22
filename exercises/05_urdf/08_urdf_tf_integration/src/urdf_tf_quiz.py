#!/usr/bin/env python3
# I AM NOT DONE
#
# 练习: urdf_tf_integration
# 模块: 05 - URDF & Robot Modeling
# 难度: ★★★☆☆
#
# 学习目标:
#   理解 URDF 如何与 TF2 系统集成，robot_state_publisher 的工作原理。
#
# 说明:
#   这是一个"探索"类练习。根据你对 URDF 和 TF2 集成的理解，
#   填写下面每个问答变量的正确答案。
#
#   你可以参考以下资料：
#   - ROS2 文档: https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/
#   - robot_state_publisher 文档
#   - joint_state_publisher 文档
#
# 步骤:
#   1. 根据 ROS2 知识填写每个 answer 变量的正确值
#   2. 填写完成后，删除文件顶部的 "# I AM NOT DONE"

# ===== 问答区域 =====

# 问题 1: robot_state_publisher 节点从哪个参数读取 URDF？
# 选项: "robot_description" / "urdf_file" / "model_path"
answer_urdf_parameter = "FILL_IN"

# 问题 2: robot_state_publisher 将 fixed joint 的变换发布到哪个话题？
# 选项: "/tf" / "/tf_static" / "/joint_states"
answer_fixed_joint_topic = "FILL_IN"

# 问题 3: 哪个节点负责发布 revolute/prismatic 等可动关节的状态？
# 选项: "robot_state_publisher" / "joint_state_publisher" / "tf2_ros"
answer_movable_joint_publisher = "FILL_IN"

# 问题 4: joint_state_publisher 发布消息到哪个话题？
# 选项: "/tf" / "/tf_static" / "/joint_states"
answer_joint_states_topic = "FILL_IN"

# 问题 5: 一个有 N 个 link 和 N-1 个 joint 的 URDF 会产生多少个 TF 帧？
# 选项: "N" / "N-1" / "2N"
answer_tf_frame_count = "FILL_IN"

# ===== 结束 =====

if __name__ == '__main__':
    print(f'answer_urdf_parameter = {answer_urdf_parameter}')
    print(f'answer_fixed_joint_topic = {answer_fixed_joint_topic}')
    print(f'answer_movable_joint_publisher = {answer_movable_joint_publisher}')
    print(f'answer_joint_states_topic = {answer_joint_states_topic}')
    print(f'answer_tf_frame_count = {answer_tf_frame_count}')
