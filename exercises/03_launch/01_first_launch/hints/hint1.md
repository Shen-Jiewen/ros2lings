# 提示 1

Launch 文件中的 `Node` action 需要正确的 `package` 和 `executable`。
`package` 是 `package.xml` 中定义的包名，
`executable` 是 `CMakeLists.txt` 中安装到 `lib/${PROJECT_NAME}` 的文件名。

另外，`LaunchDescription` 的列表里必须包含要启动的节点。
