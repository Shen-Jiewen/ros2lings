# 提示 1

自定义服务的头文件路径遵循特定的命名规则：
- 包名就是项目名（`ros2lings_19_custom_srv`）
- 服务文件名 `ComputeArea.srv` 会被转换为 `compute_area.hpp`
- 完整路径是 `包名/srv/snake_case名.hpp`

想想 CamelCase 是怎么转换成 snake_case 的？
