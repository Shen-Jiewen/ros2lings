# 提示 1

首先解决 TODO 1：创建 `StaticTransformBroadcaster`。

当前代码中 `tf_broadcaster_` 被初始化为 `nullptr`。
你需要用 `std::make_shared` 来创建一个实际的广播器对象，
并将当前节点（`this`）作为参数传入。

检查一下成员变量的类型声明，那里有你需要的类型信息。
