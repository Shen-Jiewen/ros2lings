# 提示 1

这个练习需要你完成四个 TODO 步骤。先从构建文件路径开始。

在 Python 中，`__file__` 是当前文件的路径。由于 launch 文件在 `launch/` 目录下，
而 URDF 文件在 `urdf/` 目录下，你需要：

1. 获取 launch 文件所在目录: `os.path.dirname(__file__)`
2. 向上一级到包根目录: `..`
3. 进入 urdf 目录: `urdf`
4. 指定文件名: `robot.urdf`

`os.path.join()` 可以帮你安全地拼接这些路径组件。
