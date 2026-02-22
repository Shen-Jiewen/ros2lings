# 提示 2

测试检查 `<box size="...">` 中的属性引用语法。

在 Xacro 中，引用属性（property）或宏参数（macro parameter）时，
应使用 `${}` 语法：

```xml
<!-- 正确 -->
<box size="${width} ${length} ${height}" />

<!-- 错误 -->
<box size="$(width) $(length) $(height)" />
```

`$()` 是 ROS launch 文件中的替换语法，在 Xacro 中不适用。
检查宏定义内部的 `<box>` 标签，看看属性引用用的是哪种语法。
