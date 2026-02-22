#!/usr/bin/env python3

# ===== 问答区域 =====

# 问题 1: 哪个命令行工具可以生成 TF 树的 PDF 图？
answer_view_frames_tool = "view_frames"

# 问题 2: TF2 中，静态变换发布在哪个话题上？
answer_static_topic = "/tf_static"

# 问题 3: 动态变换发布在哪个话题上？
answer_dynamic_topic = "/tf"

# 问题 4: lookupTransform(target, source, time) 返回的变换方向是？
answer_transform_direction = "source_to_target"

# 问题 5: 如果 TF 树中出现两个不同的父节点指向同一个子节点，会发生什么？
answer_multi_parent = "error"

# ===== 结束 =====

if __name__ == '__main__':
    print(f'answer_view_frames_tool = {answer_view_frames_tool}')
    print(f'answer_static_topic = {answer_static_topic}')
    print(f'answer_dynamic_topic = {answer_dynamic_topic}')
    print(f'answer_transform_direction = {answer_transform_direction}')
    print(f'answer_multi_parent = {answer_multi_parent}')
