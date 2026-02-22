# 提示 2

部分答案提示：

- TODO 1: `urdf_path = os.path.join(os.path.dirname(__file__), '..', 'urdf', 'robot.urdf')`
- TODO 2: 使用 `with open(urdf_path, 'r') as f:` 读取文件，`robot_description = f.read()`
- TODO 3: `Node()` 的 `package` 和 `executable` 都是 `'robot_state_publisher'`，
  参数通过 `parameters=[{'robot_description': robot_description}]` 传入
- TODO 4: 不要忘记把 `robot_state_pub_node` 加入 `LaunchDescription` 的列表中
