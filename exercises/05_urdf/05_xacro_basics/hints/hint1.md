# 提示 1

测试首先检查 Xacro 的命名空间 URL 是否正确。

在 Xacro 文件中，`<robot>` 标签必须声明 xacro 命名空间：
```xml
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="...">
```

仔细看看当前文件中的命名空间 URL，是不是有拼写错误？
特别注意末尾的单词是否完整。
