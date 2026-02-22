#!/usr/bin/env python3
# I AM NOT DONE
#
# 练习: tf_debug
# 模块: 04 - TF2 Transforms
# 难度: ★★☆☆☆
#
# 学习目标:
#   了解 TF2 的调试工具和核心概念，掌握 TF 树分析和帧关系的理解。
#
# 说明:
#   这是一个"探索"类练习。根据你对 TF2 系统的了解，
#   填写下面每个问答变量的正确答案。
#
#   你可以参考以下资料：
#   - ROS2 TF2 文档: https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html
#   - tf2_tools 包的命令行工具
#   - 尝试运行 ros2 run tf2_tools view_frames
#
# 步骤:
#   1. 根据 ROS2 TF2 知识填写每个 answer 变量的正确值
#   2. 填写完成后，删除文件顶部的 "# I AM NOT DONE"

# ===== 问答区域 =====

# 问题 1: 哪个命令行工具可以生成 TF 树的 PDF 图？
# 提示: 它属于 tf2_tools 包，运行方式为 ros2 run tf2_tools ___
answer_view_frames_tool = "FILL_IN"

# 问题 2: TF2 中，静态变换发布在哪个话题上？
# 提示: 以 "/" 开头的话题名
answer_static_topic = "FILL_IN"

# 问题 3: 动态变换发布在哪个话题上？
# 提示: 比静态话题名更短
answer_dynamic_topic = "FILL_IN"

# 问题 4: lookupTransform(target, source, time) 返回的变换方向是？
# 选项: "source_to_target" / "target_to_source"
answer_transform_direction = "FILL_IN"

# 问题 5: 如果 TF 树中出现两个不同的父节点指向同一个子节点，会发生什么？
# 选项: "error" / "merge" / "override"
answer_multi_parent = "FILL_IN"

# ===== 结束 =====

if __name__ == '__main__':
    print(f'answer_view_frames_tool = {answer_view_frames_tool}')
    print(f'answer_static_topic = {answer_static_topic}')
    print(f'answer_dynamic_topic = {answer_dynamic_topic}')
    print(f'answer_transform_direction = {answer_transform_direction}')
    print(f'answer_multi_parent = {answer_multi_parent}')
