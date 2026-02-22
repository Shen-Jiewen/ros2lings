# 提示 2

三个 TODO 分别需要：

1. 构建 YAML 路径：
   ```python
   yaml_path = os.path.join(
       os.path.dirname(__file__), '..', 'config', 'params.yaml'
   )
   ```

2. 在 `Node` 中添加 `parameters=[yaml_path]`

3. 把 `node` 加入 `LaunchDescription([node])`
