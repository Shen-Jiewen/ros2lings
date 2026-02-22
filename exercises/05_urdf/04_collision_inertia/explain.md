# URDF 中的碰撞几何体与惯性属性

## 概念

一个完整的 URDF link 通常包含三个子元素：

| 元素 | 用途 | 必需？ |
|------|------|--------|
| `<visual>` | 可视化外观（RViz 中显示） | 可选，但推荐 |
| `<collision>` | 碰撞检测形状（物理引擎使用） | 仿真时必需 |
| `<inertial>` | 惯性属性（质量、转动惯量） | 仿真时必需 |

在纯可视化场景中（如 RViz），只需要 `<visual>` 即可。
但如果要在 Gazebo 等物理仿真环境中使用，**必须**同时提供 `<collision>` 和 `<inertial>`。

## Collision（碰撞）

`<collision>` 的结构与 `<visual>` 类似，但它定义的是碰撞检测用的形状：

```xml
<collision>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <box size="1.0 0.5 0.3"/>
  </geometry>
</collision>
```

### 最佳实践

- **碰撞几何体通常与 visual 相同或更简单**
  - 对于简单形状，直接复用 visual 的 geometry
  - 对于复杂 mesh 模型，碰撞体通常用简化的 box/cylinder/sphere 近似
- 简化碰撞体可以提高仿真性能
- `<collision>` 中不需要 `<material>`，因为碰撞体不用于渲染

## Inertial（惯性）

`<inertial>` 定义了 link 的质量和转动惯量，物理引擎需要这些信息来计算动力学：

```xml
<inertial>
  <mass value="5.0"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <inertia ixx="0.1354" ixy="0" ixz="0"
           iyy="0.4521" iyz="0" izz="0.5417"/>
</inertial>
```

### `<mass>` 元素

```xml
<mass value="5.0"/>
```

- `value` — 质量（千克）
- 必须为正数

### `<origin>` 元素

```xml
<origin xyz="0 0 0" rpy="0 0 0"/>
```

- 定义质心相对于 link 原点的位置
- 对于均匀材质的对称形状，质心通常在几何中心

### `<inertia>` 元素

```xml
<inertia ixx="0.1354" ixy="0" ixz="0"
         iyy="0.4521" iyz="0" izz="0.5417"/>
```

这是 3x3 惯性矩阵的 6 个独立分量（矩阵是对称的）：

```
| ixx  ixy  ixz |
| ixy  iyy  iyz |
| ixz  iyz  izz |
```

- **ixx, iyy, izz**（主对角线）— 绕各轴的转动惯量，**必须为正数**
- **ixy, ixz, iyz**（非对角线）— 惯性积，对于对称物体通常为 0

### 常见形状的惯性公式

**长方体** (质量 m, 尺寸 x, y, z)：
```
ixx = (1/12) * m * (y^2 + z^2)
iyy = (1/12) * m * (x^2 + z^2)
izz = (1/12) * m * (x^2 + y^2)
```

**圆柱体** (质量 m, 半径 r, 长度 h)：
```
ixx = iyy = (1/12) * m * (3*r^2 + h^2)
izz = (1/2) * m * r^2
```

**球体** (质量 m, 半径 r)：
```
ixx = iyy = izz = (2/5) * m * r^2
```

## 完整的 Link 示例

```xml
<link name="base_link">
  <!-- 可视化 -->
  <visual>
    <geometry>
      <box size="1.0 0.5 0.3"/>
    </geometry>
    <material name="blue">
      <color rgba="0 0 1 1"/>
    </material>
  </visual>

  <!-- 碰撞 -->
  <collision>
    <geometry>
      <box size="1.0 0.5 0.3"/>
    </geometry>
  </collision>

  <!-- 惯性 -->
  <inertial>
    <mass value="5.0"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <inertia ixx="0.1354" ixy="0" ixz="0"
             iyy="0.4521" iyz="0" izz="0.5417"/>
  </inertial>
</link>
```

## 常见错误

1. **缺少 collision 导致物体穿透** — Gazebo 中物体会"掉穿"地面
2. **缺少 inertial 导致仿真异常** — 物理引擎不知道如何处理该 link
3. **mass 为 0 或负数** — 会导致仿真崩溃
4. **inertia 主对角线为 0** — 会导致数值不稳定
5. **collision 与 visual 偏移不一致** — 物体看起来碰不到实际碰撞面
