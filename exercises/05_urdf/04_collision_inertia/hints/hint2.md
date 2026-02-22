# 提示 2

测试还检查每个 link 中是否有 `<inertial>` 元素，以及其中的 `<mass>` 和 `<inertia>`。

`<inertial>` 包含三个子元素：

```xml
<inertial>
  <mass value="5.0"/>            <!-- 质量（千克） -->
  <origin xyz="0 0 0" rpy="0 0 0"/>  <!-- 质心位置 -->
  <inertia ixx="0.1354" ixy="0" ixz="0"
           iyy="0.4521" iyz="0" izz="0.5417"/>  <!-- 惯性矩阵 -->
</inertial>
```

关键要求：
- `mass` 的 `value` 必须为正数
- `inertia` 的 `ixx`、`iyy`、`izz`（主对角线）必须为正数
- 对于对称物体，`ixy`、`ixz`、`iyz` 通常设为 0
