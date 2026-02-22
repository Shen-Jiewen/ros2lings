# Module 03: Launch & Parameters — Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Add 10 new exercises teaching ROS2 launch files and parameter system, bringing total from 27 to 37.

**Architecture:** Each exercise is an independent ROS2 package under `exercises/03_launch/`, following the same CMakeLists.txt + package.xml + explain.md + hints/ + src/ + test/ pattern as existing modules. Launch-file exercises use Python launch API. Parameter exercises cover both C++ (rclcpp) and Python (rclpy). Solutions go in `solutions/03_launch/`.

**Tech Stack:** ROS2 Humble, ament_cmake, rclcpp, rclpy, launch, launch_ros, ament_cmake_pytest, ament_cmake_gtest

**Design doc:** `docs/plans/2026-02-22-modules-03-05-design.md`

**Existing patterns to follow:** See exercises in `exercises/01_nodes/` and `exercises/02_services/` for exact file formats.

---

## Task 1: Exercise 28 — 01_first_launch (fix, Python, difficulty 1)

**Concept:** Minimal Python launch file that starts a single node.

**Files to create:**
- `exercises/03_launch/01_first_launch/package.xml`
- `exercises/03_launch/01_first_launch/CMakeLists.txt`
- `exercises/03_launch/01_first_launch/explain.md`
- `exercises/03_launch/01_first_launch/hints/hint1.md`
- `exercises/03_launch/01_first_launch/hints/hint2.md`
- `exercises/03_launch/01_first_launch/hints/hint3.md`
- `exercises/03_launch/01_first_launch/src/first_launch_node.py` — simple node (correct, no bugs)
- `exercises/03_launch/01_first_launch/launch/first_launch.py` — launch file with bugs
- `exercises/03_launch/01_first_launch/test/test_first_launch.py`
- `solutions/03_launch/01_first_launch/launch/first_launch.py` — correct launch file

### Step 1: Create package.xml

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>ros2lings_28_first_launch</name>
  <version>0.1.0</version>
  <description>ROS2lings Exercise: first_launch</description>
  <maintainer email="ros2lings@example.com">ros2lings</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <exec_depend>ros2launch</exec_depend>
  <test_depend>ament_cmake_pytest</test_depend>
  <test_depend>launch_testing</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### Step 2: Create CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.8)
project(ros2lings_28_first_launch)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  src/first_launch_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  launch/first_launch.py
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_first_launch test/test_first_launch.py
    TIMEOUT 30
  )
endif()

ament_package()
```

### Step 3: Create the node (correct code, no bugs — students fix the launch file)

**File:** `src/first_launch_node.py`

```python
#!/usr/bin/env python3
"""A simple node that publishes a greeting message once."""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class FirstLaunchNode(Node):
    def __init__(self):
        super().__init__('first_launch_node')
        self.publisher_ = self.create_publisher(String, 'greeting', 10)
        self.timer_ = self.create_timer(0.5, self.timer_callback)
        self.count_ = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello from launch #{self.count_}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
        self.count_ += 1


def main(args=None):
    rclpy.init(args=args)
    node = FirstLaunchNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Step 4: Create the launch file (with bugs)

**File:** `launch/first_launch.py`

```python
# I AM NOT DONE
#
# 练习: first_launch
# 模块: 03 - Launch & Parameters
# 难度: ★☆☆☆☆
#
# 学习目标:
#   理解 ROS2 Python launch 文件的基本结构和 Node action。
#
# 说明:
#   下面的 launch 文件尝试启动一个简单的节点，但有几个错误需要修复。
#
# 步骤:
#   1. 修复 LaunchDescription 的返回——launch 文件必须返回 LaunchDescription
#   2. 修复 Node action 的 package 和 executable 参数
#   3. 修复完成后，删除文件顶部的 "# I AM NOT DONE"

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # TODO: 修复 Node action 的参数
    # BUG 1: package 名称错误（应该是 ros2lings_28_first_launch）
    # BUG 2: executable 名称错误（应该是 first_launch_node.py）
    first_node = Node(
        package='wrong_package_name',
        executable='wrong_executable',
        name='first_launch_node',
        output='screen',
    )

    # TODO: 修复返回值——应该返回 LaunchDescription 包含 first_node
    # BUG 3: 返回了空列表
    return LaunchDescription([])
```

### Step 5: Create solution launch file

**File:** `solutions/03_launch/01_first_launch/launch/first_launch.py`

```python
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    first_node = Node(
        package='ros2lings_28_first_launch',
        executable='first_launch_node.py',
        name='first_launch_node',
        output='screen',
    )

    return LaunchDescription([first_node])
```

### Step 6: Create explain.md

```markdown
# Launch 文件 — ROS2 的启动系统

## 概念

Launch 文件是 ROS2 的启动管理工具，用于同时启动和配置多个节点。
在 ROS2 中，launch 文件使用 Python 编写，比 ROS1 的 XML 格式更灵活。

## 基本结构

每个 launch 文件需要：
1. 导入 `LaunchDescription` 和需要的 actions
2. 定义 `generate_launch_description()` 函数
3. 返回一个 `LaunchDescription` 对象

## Node Action

`Node` action 是最常用的 launch action，用于启动一个 ROS2 节点：
- `package`: 节点所在的包名
- `executable`: 可执行文件名
- `name`: 节点运行时的名称（可选，覆盖默认名）
- `output`: 输出方式（'screen' 表示输出到终端）

## 关键点

- Launch 文件必须包含 `generate_launch_description()` 函数
- 该函数必须返回 `LaunchDescription` 对象
- Node action 的 package 必须与 package.xml 中的名称一致
- executable 必须与 CMakeLists.txt 中 install 的文件名一致
```

### Step 7: Create hints

**hints/hint1.md:**
```markdown
# 提示 1

Launch 文件的 `generate_launch_description()` 函数必须返回一个
`LaunchDescription` 对象，里面包含要启动的 actions。

检查一下返回的 `LaunchDescription([])` 是不是少了什么？
```

**hints/hint2.md:**
```markdown
# 提示 2

- `package` 参数必须与 `package.xml` 中的 `<name>` 标签一致
- `executable` 参数必须与 `CMakeLists.txt` 中 `install(PROGRAMS ...)` 安装的文件名一致
- 检查 `package.xml` 看看正确的包名是什么
```

**hints/hint3.md:**
```markdown
# 提示 3

完整修复：
1. `package='ros2lings_28_first_launch'`
2. `executable='first_launch_node.py'`
3. `return LaunchDescription([first_node])`
```

### Step 8: Create test file

**File:** `test/test_first_launch.py`

```python
#!/usr/bin/env python3
import pytest
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


@pytest.fixture(scope='module', autouse=True)
def init_rclpy():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_launch_file_exists():
    """测试 launch 文件是否可以被导入"""
    import importlib.util
    import os
    launch_file = os.path.join(
        os.path.dirname(__file__), '..', 'launch', 'first_launch.py'
    )
    spec = importlib.util.spec_from_file_location('first_launch', launch_file)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    assert hasattr(module, 'generate_launch_description')


def test_launch_description_not_empty():
    """测试 LaunchDescription 包含至少一个 action"""
    import importlib.util
    import os
    launch_file = os.path.join(
        os.path.dirname(__file__), '..', 'launch', 'first_launch.py'
    )
    spec = importlib.util.spec_from_file_location('first_launch', launch_file)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    ld = module.generate_launch_description()
    assert len(ld.entities) > 0, "LaunchDescription 不能为空"


def test_node_can_publish():
    """测试节点能否正常创建并发布消息"""
    node = Node('test_first_launch')
    pub = node.create_publisher(String, 'greeting', 10)
    msg = String()
    msg.data = 'test'
    pub.publish(msg)
    node.destroy_node()
```

### Step 9: Commit

```bash
git add exercises/03_launch/01_first_launch/ solutions/03_launch/01_first_launch/
git commit -m "feat(module03): add exercise 28 — first_launch"
```

---

## Task 2: Exercise 29 — 02_multi_node_launch (fix, Python, difficulty 1)

**Concept:** Launch file that starts multiple nodes from one file.

**Files:** Same structure as Task 1 but under `exercises/03_launch/02_multi_node_launch/`

**Node file:** `src/talker_node.py` — publisher, `src/listener_node.py` — subscriber (both correct)

**Launch file bugs:**
1. Second Node action missing entirely (only one node launched)
2. Nodes have conflicting names (both named 'talker')
3. Missing `output='screen'` on one node

**Solution launch:** Correct launch with both nodes, unique names, output='screen'

**Test:** Verify launch description has 2 entities, verify both nodes can be created, verify pub/sub can communicate.

**Package name:** `ros2lings_29_multi_node_launch`

### Commit
```bash
git add exercises/03_launch/02_multi_node_launch/ solutions/03_launch/02_multi_node_launch/
git commit -m "feat(module03): add exercise 29 — multi_node_launch"
```

---

## Task 3: Exercise 30 — 03_launch_arguments (implement, Python, difficulty 2)

**Concept:** DeclareLaunchArgument and LaunchConfiguration for parameterizing launches.

**Files:** Same structure under `exercises/03_launch/03_launch_arguments/`

**Node file:** `src/configurable_node.py` — node that reads a `topic_name` parameter and publishes to it (correct)

**Launch file skeleton:** Has TODO placeholders for:
1. `DeclareLaunchArgument('topic_name', default_value='hello')`
2. `LaunchConfiguration('topic_name')` to pass to Node's parameters
3. Passing the configuration to the Node action via `parameters=[{'topic_name': topic_config}]`

**Solution launch:** Complete launch with argument declaration and configuration passing.

**Test:** Verify launch description has DeclareLaunchArgument entity, verify node accepts parameter.

**Package name:** `ros2lings_30_launch_arguments`

### Commit
```bash
git add exercises/03_launch/03_launch_arguments/ solutions/03_launch/03_launch_arguments/
git commit -m "feat(module03): add exercise 30 — launch_arguments"
```

---

## Task 4: Exercise 31 — 04_node_parameters (fix, C++, difficulty 2)

**Concept:** declare_parameter, get_parameter, parameter types in C++ nodes.

**Files:** Same structure under `exercises/03_launch/04_node_parameters/`

**Source file bugs (src/node_parameters.cpp):**
1. `declare_parameter` called with wrong type (string instead of int)
2. `get_parameter` returns wrong type
3. Missing parameter declaration for one parameter
4. Parameter name mismatch between declare and get

**Solution:** Correct parameter declarations and retrieval.

**Test (C++ gtest):**
- Test node can be created
- Test parameter has correct default value
- Test parameter can be set and retrieved
- Test parameter type is correct

**Package name:** `ros2lings_31_node_parameters`
**Dependencies:** `rclcpp`, `ament_cmake_gtest`

### Key source pattern:

```cpp
// Exercise (buggy):
class ParameterNode : public rclcpp::Node {
public:
  ParameterNode() : Node("parameter_node") {
    // BUG: 类型不匹配，声明为 string 但应该是 int
    this->declare_parameter("max_speed", "fast");
    // BUG: 参数名拼写错误
    this->declare_parameter("robot_name", "ros2bot");
    // TODO: 缺少 update_frequency 参数声明

    auto speed = this->get_parameter("max_speed").as_int();
    auto name = this->get_parameter("robot_nam").as_string(); // BUG: typo
  }
};
```

```cpp
// Solution:
class ParameterNode : public rclcpp::Node {
public:
  ParameterNode() : Node("parameter_node") {
    this->declare_parameter("max_speed", 10);
    this->declare_parameter("robot_name", "ros2bot");
    this->declare_parameter("update_frequency", 30.0);

    auto speed = this->get_parameter("max_speed").as_int();
    auto name = this->get_parameter("robot_name").as_string();
    auto freq = this->get_parameter("update_frequency").as_double();
    RCLCPP_INFO(get_logger(), "Speed: %ld, Name: %s, Freq: %.1f", speed, name.c_str(), freq);
  }
};
```

### Commit
```bash
git add exercises/03_launch/04_node_parameters/ solutions/03_launch/04_node_parameters/
git commit -m "feat(module03): add exercise 31 — node_parameters"
```

---

## Task 5: Exercise 32 — 05_parameter_callback (fix, C++, difficulty 2)

**Concept:** on_set_parameters_callback for dynamic parameter updates.

**Source file bugs:**
1. Callback signature wrong (missing `const` on vector reference)
2. Callback not registered with `add_on_set_parameters_callback`
3. Return value incorrect (should return `rcl_interfaces::msg::SetParametersResult`)

**Test:** Verify parameter can be changed dynamically, callback fires, node reflects new value.

**Package name:** `ros2lings_32_parameter_callback`
**Dependencies:** `rclcpp`, `rcl_interfaces`, `ament_cmake_gtest`

### Commit
```bash
git add exercises/03_launch/05_parameter_callback/ solutions/03_launch/05_parameter_callback/
git commit -m "feat(module03): add exercise 32 — parameter_callback"
```

---

## Task 6: Exercise 33 — 06_yaml_config (implement, Python, difficulty 2)

**Concept:** Loading parameters from YAML files via launch.

**Files:** Adds `config/params.yaml` file to exercise.

**Launch file skeleton:** TODO placeholders for loading YAML config and passing to node.

**YAML config (correct):**
```yaml
configurable_node:
  ros__parameters:
    robot_name: "ros2bot"
    max_speed: 1.5
    enable_logging: true
```

**Node file:** Python node that declares and logs these parameters.

**Test:** Verify node accepts YAML parameters, values match config.

**Package name:** `ros2lings_33_yaml_config`

### Commit
```bash
git add exercises/03_launch/06_yaml_config/ solutions/03_launch/06_yaml_config/
git commit -m "feat(module03): add exercise 33 — yaml_config"
```

---

## Task 7: Exercise 34 — 07_conditional_launch (implement, Python, difficulty 3)

**Concept:** IfCondition, UnlessCondition, GroupAction for conditional node startup.

**Launch file skeleton:** TODO placeholders for:
1. `DeclareLaunchArgument('use_sim', default_value='false')`
2. `IfCondition(LaunchConfiguration('use_sim'))` wrapping a sim node
3. `UnlessCondition(LaunchConfiguration('use_sim'))` wrapping a real node
4. `GroupAction` with namespace

**Test:** Verify launch description has conditional actions, different argument values produce different entity counts.

**Package name:** `ros2lings_34_conditional_launch`

### Commit
```bash
git add exercises/03_launch/07_conditional_launch/ solutions/03_launch/07_conditional_launch/
git commit -m "feat(module03): add exercise 34 — conditional_launch"
```

---

## Task 8: Exercise 35 — 08_parameter_py (fix, Python, difficulty 2)

**Concept:** Python node parameter declaration and usage with rclpy.

**Source file bugs:**
1. `declare_parameter` called after `get_parameter` (order wrong)
2. Wrong default value type
3. Missing parameter descriptor

**Test:** Verify parameters declared correctly, can be retrieved, have correct types.

**Package name:** `ros2lings_35_parameter_py`

### Commit
```bash
git add exercises/03_launch/08_parameter_py/ solutions/03_launch/08_parameter_py/
git commit -m "feat(module03): add exercise 35 — parameter_py"
```

---

## Task 9: Exercise 36 — 09_launch_composition (implement, Python, difficulty 3)

**Concept:** ComposableNodeContainer and LoadComposableNode for component composition via launch.

**Launch file skeleton:** TODO placeholders for:
1. `ComposableNodeContainer` with container name and namespace
2. `ComposableNode` entries for publisher and subscriber components
3. Correct plugin names matching the component registration

**Node files:** Two C++ component nodes (publisher_component.cpp, subscriber_component.cpp) — correct code.

**Test:** Verify launch description has container action, container has composable nodes.

**Package name:** `ros2lings_36_launch_composition`
**Dependencies:** `rclcpp`, `rclcpp_components`, `std_msgs`, `ament_cmake_gtest`

### Commit
```bash
git add exercises/03_launch/09_launch_composition/ solutions/03_launch/09_launch_composition/
git commit -m "feat(module03): add exercise 36 — launch_composition"
```

---

## Task 10: Exercise 37 — 10_launch_events (explore, Python, difficulty 3)

**Concept:** OnProcessExit, RegisterEventHandler, event-driven launch patterns.

**Source file pattern (explore mode):** Global string variables with questions about launch events:
1. Which event fires when a process exits? → "OnProcessExit"
2. Which handler registers event callbacks? → "RegisterEventHandler"
3. Can you chain launches (start B after A exits)? → "yes"
4. Which action shuts down the launch after a node exits? → "Shutdown"
5. What class wraps a launch action with a condition? → "EventHandler"

**Test:** Verify answers match expected values.

**Package name:** `ros2lings_37_launch_events`

### Commit
```bash
git add exercises/03_launch/10_launch_events/ solutions/03_launch/10_launch_events/
git commit -m "feat(module03): add exercise 37 — launch_events"
```

---

## Task 11: Update info.toml

**File to modify:** `info.toml`

Append 10 new exercise entries after the Module 02 section:

```toml
# ── Module 03: Launch & Parameters (10 exercises) ──

[[exercises]]
name = "28_first_launch"
dir = "03_launch/01_first_launch"
module = "Launch & Parameters"
mode = "fix"
language = "python"
difficulty = 1
estimated_minutes = 10
hint_count = 3
test = true
hint = "launch 文件必须返回 LaunchDescription，Node action 需要正确的 package 和 executable。"

[[exercises]]
name = "29_multi_node_launch"
dir = "03_launch/02_multi_node_launch"
module = "Launch & Parameters"
mode = "fix"
language = "python"
difficulty = 1
estimated_minutes = 10
hint_count = 3
depends_on = ["28_first_launch"]
test = true
hint = "每个节点需要唯一的 name，确保所有节点都添加到 LaunchDescription 中。"

[[exercises]]
name = "30_launch_arguments"
dir = "03_launch/03_launch_arguments"
module = "Launch & Parameters"
mode = "implement"
language = "python"
difficulty = 2
estimated_minutes = 15
hint_count = 3
depends_on = ["29_multi_node_launch"]
test = true
hint = "用 DeclareLaunchArgument 声明参数，用 LaunchConfiguration 获取参数值。"

[[exercises]]
name = "31_node_parameters"
dir = "03_launch/04_node_parameters"
module = "Launch & Parameters"
mode = "fix"
language = "cpp"
difficulty = 2
estimated_minutes = 15
hint_count = 3
test = true
hint = "declare_parameter 的默认值类型必须与 get_parameter 的 as_xxx() 类型匹配。"

[[exercises]]
name = "32_parameter_callback"
dir = "03_launch/05_parameter_callback"
module = "Launch & Parameters"
mode = "fix"
language = "cpp"
difficulty = 2
estimated_minutes = 15
hint_count = 3
depends_on = ["31_node_parameters"]
test = true
hint = "用 add_on_set_parameters_callback 注册回调，返回 SetParametersResult。"

[[exercises]]
name = "33_yaml_config"
dir = "03_launch/06_yaml_config"
module = "Launch & Parameters"
mode = "implement"
language = "python"
difficulty = 2
estimated_minutes = 15
hint_count = 3
depends_on = ["30_launch_arguments"]
test = true
hint = "在 launch 文件中用 parameters=[yaml_file_path] 传入 YAML 配置文件路径。"

[[exercises]]
name = "34_conditional_launch"
dir = "03_launch/07_conditional_launch"
module = "Launch & Parameters"
mode = "implement"
language = "python"
difficulty = 3
estimated_minutes = 20
hint_count = 3
depends_on = ["30_launch_arguments"]
test = true
hint = "IfCondition 接受 LaunchConfiguration，UnlessCondition 是它的反面。"

[[exercises]]
name = "35_parameter_py"
dir = "03_launch/08_parameter_py"
module = "Launch & Parameters"
mode = "fix"
language = "python"
difficulty = 2
estimated_minutes = 10
hint_count = 3
test = true
hint = "必须先 declare_parameter 再 get_parameter，默认值类型要匹配。"

[[exercises]]
name = "36_launch_composition"
dir = "03_launch/09_launch_composition"
module = "Launch & Parameters"
mode = "implement"
language = "python"
difficulty = 3
estimated_minutes = 20
hint_count = 3
depends_on = ["29_multi_node_launch"]
test = true
hint = "ComposableNodeContainer 管理组件，ComposableNode 的 plugin 名必须与 REGISTER_NODE 宏一致。"

[[exercises]]
name = "37_launch_events"
dir = "03_launch/10_launch_events"
module = "Launch & Parameters"
mode = "explore"
language = "python"
difficulty = 3
estimated_minutes = 15
hint_count = 3
depends_on = ["29_multi_node_launch"]
test = true
hint = "OnProcessExit 在进程退出后触发，RegisterEventHandler 注册事件处理器。"
```

Also update `final_message` to mention Launch & Parameters.

### Commit
```bash
git add info.toml
git commit -m "feat(module03): add 10 exercises to info.toml"
```

---

## Task 12: Update CI workflow

**File to modify:** `.github/workflows/ci.yml`

### Changes:

1. Add new apt dependencies to the Install step:
```yaml
ros-humble-launch-ros \
ros-humble-launch-testing \
ros-humble-launch-testing-ament-cmake \
```

2. Add `exercises/03_launch/*` to the colcon build paths step:
```yaml
colcon build \
  --paths exercises/01_nodes/* exercises/02_services/* exercises/03_launch/* \
```

3. Solution overlay script already auto-discovers `solutions/*/` — no change needed.

### Commit
```bash
git add .github/workflows/ci.yml
git commit -m "ci: add Module 03 dependencies and build paths"
```

---

## Task 13: Update README.md

**File to modify:** `README.md`

Add Module 03 exercise table after the Module 02 table:

```markdown
### Module 03: Launch & Parameters (10 exercises)

| # | Exercise | Language | Concepts |
|---|----------|----------|----------|
| 1 | `01_first_launch` | Python | Launch file basics, Node action |
| 2 | `02_multi_node_launch` | Python | Multi-node launch |
| 3 | `03_launch_arguments` | Python | Launch arguments, configuration |
| 4 | `04_node_parameters` | C++ | Parameter declaration and retrieval |
| 5 | `05_parameter_callback` | C++ | Dynamic parameter updates |
| 6 | `06_yaml_config` | Python | YAML parameter files |
| 7 | `07_conditional_launch` | Python | Conditional launch, GroupAction |
| 8 | `08_parameter_py` | Python | Python parameters |
| 9 | `09_launch_composition` | Python | Component composition via launch |
| 10 | `10_launch_events` | Python | Event-driven launch |
```

### Commit
```bash
git add README.md
git commit -m "docs: add Module 03 to README exercise table"
```

---

## Task 14: Push and verify CI

### Steps:
1. `git push`
2. `gh run watch` — monitor CI
3. Fix any failures (common: missing apt packages, path issues, test timeouts)
4. Iterate until both jobs are green

---

## Verification Checklist

- [ ] All 10 exercise directories exist under `exercises/03_launch/`
- [ ] All 10 solution directories exist under `solutions/03_launch/`
- [ ] `info.toml` has 37 exercises total
- [ ] `cargo test` passes (37 Rust tests)
- [ ] Local `colcon build` of new exercises succeeds with solutions overlaid
- [ ] Local `colcon test` of new exercises passes
- [ ] CI both jobs green
- [ ] README reflects all 37 exercises
