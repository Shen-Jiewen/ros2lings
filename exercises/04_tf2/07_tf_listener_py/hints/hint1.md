# 提示 1

Python 中使用 TF2 监听变换需要两个组件配合：
- `tf2_ros.Buffer` — 存储变换数据
- `tf2_ros.TransformListener` — 订阅话题并填充 Buffer

`TransformListener` 创建时必须传入 `Buffer` 实例，否则它会使用自己内部的 buffer，
导致你后续查询的 buffer 中没有数据。

另外，检查：
- `lookup_transform` 的两个帧参数顺序是否正确？（target 在前，source 在后）
- 异常处理捕获的是什么类型？TF2 抛出的异常类型是什么？
