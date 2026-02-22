# URDF 中的 Link 与 Joint

## 概念

一个真实的机器人由多个刚性部件组成，这些部件通过关节连接在一起。
在 URDF 中：
- **Link**（连杆）表示刚性部件
- **Joint**（关节）表示两个 link 之间的连接方式

## Link 与 Joint 的关系

URDF 模型形成一棵**树状结构**：

```
base_link
    │
    ├── joint: base_to_upper (revolute)
    │       │
    │       └── upper_arm
    │               │
    │               ├── joint: upper_to_lower (fixed)
    │               │       │
    │               │       └── lower_arm
```

- 每个 joint 连接一个**父 link**（parent）和一个**子 link**（child）
- 父 link 在树中更靠近根节点
- 整棵树的根是 `base_link`

## Joint 的定义

```xml
<joint name="base_to_upper" type="revolute">
  <parent link="base_link"/>
  <child link="upper_arm"/>
  <origin xyz="0 0 0.5" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
</joint>
```

### 必需元素

| 元素 | 说明 |
|------|------|
| `name` 属性 | joint 的唯一名称 |
| `type` 属性 | 关节类型（见下文） |
| `<parent link="..."/>` | 父 link 的名称 |
| `<child link="..."/>` | 子 link 的名称 |

### 常用 Joint 类型

| 类型 | 说明 | 需要 axis? | 需要 limit? |
|------|------|-----------|------------|
| `fixed` | 固定连接，无运动 | 否 | 否 |
| `revolute` | 绕轴旋转，有角度限制 | **是** | **是** |
| `continuous` | 绕轴旋转，无角度限制 | **是** | 否 |
| `prismatic` | 沿轴平移，有距离限制 | **是** | **是** |

### `<axis>` 元素

指定 joint 的运动轴方向：

```xml
<axis xyz="0 0 1"/>  <!-- 绕 z 轴旋转 -->
<axis xyz="0 1 0"/>  <!-- 绕 y 轴旋转 -->
<axis xyz="1 0 0"/>  <!-- 绕 x 轴旋转 -->
```

### `<limit>` 元素

指定 joint 的运动范围和力矩限制：

```xml
<limit lower="-1.57"   <!-- 最小角度/位移（弧度/米） -->
       upper="1.57"    <!-- 最大角度/位移（弧度/米） -->
       effort="100"    <!-- 最大力矩/力（Nm/N） -->
       velocity="1.0"  <!-- 最大速度（rad/s 或 m/s） -->
/>
```

**重要**: `revolute` 和 `prismatic` 类型的 joint **必须**包含 `<axis>` 和 `<limit>` 元素，
否则 URDF 解析器会报错。

### `<origin>` 元素

指定子 link 相对于父 link 的初始位姿：

```xml
<origin xyz="0 0 0.5" rpy="0 0 0"/>
<!-- xyz: 平移（米），rpy: 旋转 roll/pitch/yaw（弧度） -->
```

## Parent 和 Child 的命名一致性

Joint 中的 `<parent link="..."/>` 和 `<child link="..."/>` 引用的名称
**必须**与已定义的 `<link name="..."/>` 完全匹配。

```xml
<!-- 定义 link -->
<link name="upper_arm"/>

<!-- joint 中正确引用 -->
<parent link="upper_arm"/>   <!-- 正确 -->
<parent link="upper"/>       <!-- 错误！名称不匹配 -->
```

## 常见错误

1. **type 拼写错误** — `"revolut"` 不是有效类型，应为 `"revolute"`
2. **parent/child 名称不匹配** — 必须与 `<link name="...">` 完全一致
3. **revolute joint 缺少 axis/limit** — 这两个元素是 revolute 类型的必需元素
4. **树状结构被破坏** — 每个 link 只能作为一个 joint 的 child
