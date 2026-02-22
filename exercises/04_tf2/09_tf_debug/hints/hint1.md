# 提示 1

TF2 的调试工具和话题都是 ROS2 TF2 系统的标准组件：

- 问题 1: `tf2_tools` 包中有一个工具，运行后会生成一个 PDF 文件来展示帧树
- 问题 2 和 3: TF2 使用两个话题传输数据，一个用于静态变换，一个用于动态变换
- 问题 4: 想想你调用 `lookupTransform(target, source, time)` 时，
  返回的变换是用来把数据从哪里转到哪里的

你可以尝试运行 `ros2 topic list` 查看 TF 相关的话题。
