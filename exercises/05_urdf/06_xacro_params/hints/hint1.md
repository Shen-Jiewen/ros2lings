# 提示 1

第一步是定义 `xacro:property`。

属性用于存储可复用的数值，语法如下：

```xml
<xacro:property name="属性名" value="值" />
```

你需要定义两个属性：
- `arm_length`：臂段长度，值为 `0.5`
- `arm_radius`：臂段半径，值为 `0.05`

把它们放在 `<robot>` 标签内、宏定义之前。
