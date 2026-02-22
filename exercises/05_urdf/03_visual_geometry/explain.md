# URDF 中的几何体与材质颜色

## 概念

在 URDF 中，每个 link 的 `<visual>` 元素定义了它在可视化工具（如 RViz）中的外观。
`<visual>` 包含两个核心子元素：
- **`<geometry>`** — 定义形状
- **`<material>`** — 定义颜色/外观

## 基本几何体

URDF 提供四种基本几何体类型：

### Box（长方体）

```xml
<geometry>
  <box size="1.0 0.5 0.3"/>
</geometry>
```

`size` 属性包含 3 个值，分别对应 x（长）、y（宽）、z（高）方向的尺寸，单位为米。

### Cylinder（圆柱体）

```xml
<geometry>
  <cylinder radius="0.2" length="0.6"/>
</geometry>
```

- `radius` — 圆柱体的半径（米）
- `length` — 圆柱体的高度（米）
- 圆柱体默认沿 z 轴方向

### Sphere（球体）

```xml
<geometry>
  <sphere radius="0.15"/>
</geometry>
```

- `radius` — 球体的半径（米）

### Mesh（网格模型）

```xml
<geometry>
  <mesh filename="package://my_robot/meshes/part.stl" scale="1.0 1.0 1.0"/>
</geometry>
```

- `filename` — 指向 3D 模型文件（STL、DAE 等）
- `scale` — 可选，缩放比例

## Material（材质颜色）

```xml
<material name="blue">
  <color rgba="0 0 1 1"/>
</material>
```

- `name` — 材质名称，方便在多个 link 中复用
- `rgba` — 红、绿、蓝、透明度，每个值范围 0.0~1.0

### 常用颜色示例

| 颜色 | rgba 值 |
|------|---------|
| 红色 | `1 0 0 1` |
| 绿色 | `0 1 0 1` |
| 蓝色 | `0 0 1 1` |
| 白色 | `1 1 1 1` |
| 灰色 | `0.5 0.5 0.5 1` |
| 半透明红 | `1 0 0 0.5` |

## 完整的 Visual 示例

```xml
<link name="base_link">
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="1.0 0.5 0.1"/>
    </geometry>
    <material name="blue">
      <color rgba="0 0 1 1"/>
    </material>
  </visual>
</link>
```

### `<origin>` 元素

`<visual>` 中的 `<origin>` 用于调整几何体相对于 link 原点的位置和旋转：

```xml
<origin xyz="0 0 0.3" rpy="0 0 0"/>
```

- `xyz` — 平移量（米）
- `rpy` — 绕 x/y/z 轴的旋转（弧度）

## 组合多种几何体构建机器人

一个完整的机器人通常由多个不同几何体的 link 组成，通过 joint 连接：

```
base_link (box)
    |
    +-- base_to_body (fixed joint)
            |
            body (cylinder)
                |
                +-- body_to_head (fixed joint)
                        |
                        head (sphere)
```

每个 link 使用最适合其形状的几何体类型，并赋予不同的颜色以便区分。

## 常见错误

1. **`<geometry>` 内缺少具体形状** — `<geometry>` 内部必须恰好包含一个形状元素
2. **box 的 size 值个数不对** — 必须是 3 个值
3. **cylinder 缺少 radius 或 length** — 两个属性都是必需的
4. **material 中缺少 color 元素** — 只有 name 不够，还需要定义具体颜色
5. **rgba 值超出范围** — 每个值应在 0.0 到 1.0 之间
