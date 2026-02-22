# 提示 3

以下是所有 TODO 的完整解答。

**base_link** 需要添加的 collision 和 inertial（在 `</visual>` 之后）：

```xml
<collision>
  <geometry>
    <box size="1.0 0.5 0.3"/>
  </geometry>
</collision>

<inertial>
  <mass value="5.0"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <inertia ixx="0.1354" ixy="0" ixz="0"
           iyy="0.4521" iyz="0" izz="0.5417"/>
</inertial>
```

**wheel** 需要添加的 collision 和 inertial（在 `</visual>` 之后）：

```xml
<collision>
  <geometry>
    <cylinder radius="0.1" length="0.05"/>
  </geometry>
</collision>

<inertial>
  <mass value="0.5"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <inertia ixx="0.0005" ixy="0" ixz="0"
           iyy="0.0005" iyz="0" izz="0.0005"/>
</inertial>
```

完成后记得删除文件顶部的 `<!-- I AM NOT DONE -->` 注释。
