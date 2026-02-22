# 提示 1

在 Launch 文件中加载 YAML 配置文件，关键是构建正确的文件路径。
`os.path.dirname(__file__)` 返回当前 Launch 文件所在的目录，
然后用 `os.path.join` 拼接出 YAML 文件的路径。

加载方式是在 `Node` 的 `parameters` 参数中传入 YAML 文件路径。
