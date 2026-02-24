# ROS2lings

[![CI](https://github.com/Shen-Jiewen/ros2lings/actions/workflows/ci.yml/badge.svg)](https://github.com/Shen-Jiewen/ros2lings/actions/workflows/ci.yml)

**通过修复坏掉的代码来学习 ROS2** — 类似 [rustlings](https://github.com/rust-lang/rustlings)，但面向 ROS2。

[English](README.md)

## ROS2lings 是什么？

ROS2lings 是一个交互式、动手实践的 ROS2 学习平台。它提供了 **55 个渐进式练习**，分布在 5 个模块中，通过让你修复"坏掉的"代码来教你 ROS2 核心概念。每个练习都是一个真实的 ROS2 包，带有 `TODO` 标记指引你需要修改的地方。基于 Rust 的 CLI 工具会监视你的编辑、自动重新构建、运行测试，并自动推进你的学习进度。

**灵感来自 [rustlings](https://github.com/rust-lang/rustlings)** — 如果你用过 rustlings 学 Rust，你会感到非常熟悉。

## 学习流程

```
┌─────────────────────────────────────────────────────────┐
│  1. ros2lings 给你展示一个有问题的练习                     │
│  2. 阅读注释，理解 TODO 要求                               │
│  3. 在编辑器中修复代码                                     │
│  4. ros2lings 自动检测到你的修改                           │
│  5. 它自动用 colcon 重新构建并运行测试                      │
│  6. 测试通过 → 自动进入下一个练习！                         │
│  7. 卡住了？按 'h' 查看渐进式提示                          │
└─────────────────────────────────────────────────────────┘
```

每个练习文件长这样：

```cpp
// I AM NOT DONE          ← 完成后删除这行
//
// 练习: hello_node
// 模块: 01 - Nodes & Topics
// 难度: ★☆☆☆☆
//
// 步骤:
//   1. 修复 rclcpp::init() 的参数
//   2. 修复节点的创建方式（提示：需要智能指针）
//   3. 修复完成后，删除文件顶部的 "// I AM NOT DONE"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  // TODO: rclcpp::init() 需要 argc 和 argv
  rclcpp::init();

  // TODO: 节点需要用 std::make_shared 创建
  auto node = rclcpp::Node("hello_node");
  ...
}
```

## 环境要求

| 依赖 | 版本 |
|------|------|
| **Ubuntu** | 22.04 LTS |
| **ROS2** | Humble Hawksbill |
| **Rust** | 稳定版工具链 |
| **C++ 构建工具** | `build-essential`, `cmake` |
| **Python** | 3.10+ |

## 安装步骤

```bash
# 1. 确保已安装 ROS2 Humble
#    参见: https://docs.ros.org/en/humble/Installation.html

# 2. 安装 Rust（如果尚未安装）
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh

# 3. 克隆仓库
git clone https://github.com/Shen-Jiewen/ros2lings.git
cd ros2lings

# 4. 构建并安装 CLI 工具
cargo install --path .

# 5. 加载 ROS2 环境
source /opt/ros/humble/setup.bash
```

## 快速开始

```bash
# 加载 ROS2 环境（每次打开新终端都需要执行）
source /opt/ros/humble/setup.bash

# 开始学习！这会启动交互式监视模式
ros2lings
```

就这么简单！CLI 会展示你的第一个练习。在编辑器中打开文件，修复 TODO 项，保存，然后看它自动验证。

## CLI 命令

| 命令 | 说明 |
|------|------|
| `ros2lings` | 启动交互式监视模式（主学习循环） |
| `ros2lings list` | 显示所有练习及完成状态 |
| `ros2lings hint` | 显示当前练习的下一条提示 |
| `ros2lings verify` | 手动验证当前练习 |
| `ros2lings verify <名称>` | 验证指定练习 |
| `ros2lings reset <名称>` | 将练习重置为初始状态 |
| `ros2lings explain` | 显示当前练习的架构说明 |
| `ros2lings graph` | 显示学习进度概览 |

## 监视模式快捷键

| 按键 | 功能 |
|------|------|
| `q` | 退出 |
| `h` | 查看提示（再按查看下一条，最多 3 条） |
| `e` | 查看架构说明 |
| `l` | 查看当前练习信息 |
| `n` | 跳过当前练习 |

## 详细学习指南

### 第一步：了解项目结构

每个练习都是 `exercises/` 目录下的一个独立 ROS2 包，按模块组织：

```
exercises/
├── 01_nodes/          ← 节点与话题（从这里开始！）
├── 02_services/       ← 服务与动作
├── 03_launch/         ← Launch 文件与参数
├── 04_tf2/            ← TF2 坐标变换
├── 05_urdf/           ← URDF 机器人建模
└── ros2lings_interfaces/  ← 共享的自定义消息定义
```

每个练习包包含：

```
exercises/01_nodes/01_hello_node/
├── src/hello_node.cpp    ← 需要你修复的代码
├── test/                 ← 自动化测试（验证你的修复）
├── explain.md            ← 概念讲解，含架构图
├── hints/                ← 3 个渐进式提示
│   ├── hint1.md          ← 轻微提示
│   ├── hint2.md          ← 更具体的引导
│   └── hint3.md          ← 接近答案的帮助
├── CMakeLists.txt        ← 构建配置（通常不需要修改）
└── package.xml           ← 包的元数据
```

### 第二步：启动监视模式

```bash
source /opt/ros/humble/setup.bash
ros2lings
```

CLI 会显示第一个未完成的练习，并告诉你要编辑哪个文件。

### 第三步：先阅读，后编码

在动手写代码之前，从头到尾阅读练习文件。注释中包含：
- **学习目标** — 完成这个练习后你会理解什么
- **说明** — 背景和上下文
- **步骤** — 你需要做的具体操作
- **TODO 标记** — 需要修复的具体位置

同时阅读练习目录中的 `explain.md` 文件，获得更深入的概念理解。

### 第四步：修复代码

在你喜欢的编辑器（VS Code、vim 等）中打开源文件，按照 TODO 提示逐个修复。

### 第五步：善用提示

如果卡住了，在监视模式下按 `h`。提示是渐进式的：
- **提示 1**：方向性的轻微暗示
- **提示 2**：更具体的引导
- **提示 3**：几乎告诉你答案

尽量用最少的提示来解决问题！

### 第六步：移除完成标记

当所有 TODO 都修复完毕，并且你对解答有信心时，删除文件顶部的 `// I AM NOT DONE` 行。这表示你已经完成了这个练习。

### 第七步：观察自动验证

保存后，ros2lings 会：
1. 用 `colcon build` 构建你的练习
2. 用 `colcon test` 运行自动化测试
3. 全部通过 → 自动进入下一个练习！
4. 有失败 → 显示错误信息，你可以继续修改

### 第八步：查看学习进度

```bash
ros2lings list     # 查看所有练习和完成状态
ros2lings graph    # 可视化进度概览
```

### 第九步：参考解答

如果你完全卡住了，参考解答在 `solutions/` 目录中，结构与练习目录一一对应：

```
solutions/
├── 01_nodes/01_hello_node/src/hello_node.cpp
├── 01_nodes/02_first_publisher/src/first_publisher.cpp
└── ...
```

**请先尝试自己解决！** 解答是用来学习的，不是用来复制的。

## 练习模式

练习有 4 种模式，各有不同的教学方式：

| 模式 | 数量 | 说明 |
|------|------|------|
| **Fix（修复）** | 26 | 代码有 bug — 找到并修复它们 |
| **Implement（实现）** | 20 | 部分代码 — 补全缺失的部分 |
| **Explore（探索）** | 6 | 可运行的代码 — 修改它来理解行为 |
| **Debug（调试）** | 3 | 隐蔽的 bug — 诊断并修复 |

## 推荐学习路径

练习按顺序设计，每个模块都建立在前一个模块的基础上。

| 周次 | 模块 | 练习数 | 你将学到 |
|------|------|--------|----------|
| 1-2 | **01: 节点与话题** | 15 | 节点、发布者、订阅者、自定义消息、QoS、生命周期、组件化 |
| 3-4 | **02: 服务与动作** | 12 | 服务端/客户端、动作服务器/客户端、自定义接口 |
| 5-6 | **03: Launch 与参数** | 10 | Launch 文件、参数系统、YAML 配置、组件组合、事件 |
| 7-8 | **04: TF2 坐标变换** | 10 | 静态/动态变换、帧链、多机器人 TF |
| 9-10 | **05: URDF 机器人建模** | 8 | URDF、Xacro、robot_state_publisher、TF 集成 |

## 练习列表

### 模块 01：节点与话题（15 个练习）

| # | 练习 | 语言 | 难度 | 核心概念 |
|---|------|------|------|----------|
| 1 | `01_hello_node` | C++ | ★☆☆☆☆ | 节点创建、`rclcpp::init` |
| 2 | `02_first_publisher` | C++ | ★☆☆☆☆ | 发布者、定时器回调 |
| 3 | `03_first_subscriber` | C++ | ★☆☆☆☆ | 订阅者、消息回调 |
| 4 | `04_pubsub_connect` | C++ | ★★☆☆☆ | 发布/订阅连接 |
| 5 | `05_custom_message` | C++ | ★★☆☆☆ | 自定义 `.msg` 消息 |
| 6 | `06_multi_topic` | C++ | ★★☆☆☆ | 多话题发布/订阅 |
| 7 | `07_topic_introspection` | C++ | ★★☆☆☆ | 话题列表、类型检查 |
| 8 | `08_node_lifecycle_basics` | C++ | ★★★☆☆ | 生命周期节点状态机 |
| 9 | `09_hello_node_py` | Python | ★☆☆☆☆ | Python 节点基础 |
| 10 | `10_publisher_py` | Python | ★☆☆☆☆ | Python 发布者 |
| 11 | `11_subscriber_py` | Python | ★☆☆☆☆ | Python 订阅者 |
| 12 | `12_qos_mismatch` | C++ | ★★★☆☆ | QoS 配置兼容性 |
| 13 | `13_component_node` | C++ | ★★★☆☆ | 组件节点、组合 |
| 14 | `14_namespace_remap` | C++ | ★★☆☆☆ | 命名空间、重映射 |
| 15 | `15_multi_node_process` | C++ | ★★☆☆☆ | 单进程多节点 |

### 模块 02：服务与动作（12 个练习）

| # | 练习 | 语言 | 难度 | 核心概念 |
|---|------|------|------|----------|
| 1 | `01_first_service` | C++ | ★☆☆☆☆ | 服务端 |
| 2 | `02_service_client` | C++ | ★★☆☆☆ | 服务客户端、异步调用 |
| 3 | `03_service_pair` | C++ | ★★☆☆☆ | 客户端-服务端配对 |
| 4 | `04_custom_srv` | C++ | ★★☆☆☆ | 自定义 `.srv` 定义 |
| 5 | `05_service_py` | Python | ★★☆☆☆ | Python 服务 |
| 6 | `06_first_action_server` | C++ | ★★★☆☆ | 动作服务器 |
| 7 | `07_action_client` | C++ | ★★★☆☆ | 动作客户端 |
| 8 | `08_action_complete` | C++ | ★★★☆☆ | 完整动作流程 |
| 9 | `09_custom_action` | C++ | ★★★☆☆ | 自定义 `.action` 定义 |
| 10 | `10_action_py` | Python | ★★★☆☆ | Python 动作 |
| 11 | `11_service_introspection` | C++ | ★★☆☆☆ | 服务内省 |
| 12 | `12_action_state_machine` | C++ | ★★★★☆ | 动作状态机 |

### 模块 03：Launch 与参数（10 个练习）

| # | 练习 | 语言 | 难度 | 核心概念 |
|---|------|------|------|----------|
| 1 | `01_first_launch` | Python | ★☆☆☆☆ | Launch 文件基础 |
| 2 | `02_multi_node_launch` | Python | ★★☆☆☆ | 多节点启动 |
| 3 | `03_launch_arguments` | Python | ★★☆☆☆ | 启动参数配置 |
| 4 | `04_node_parameters` | C++ | ★★☆☆☆ | 参数声明与获取 |
| 5 | `05_parameter_callback` | C++ | ★★★☆☆ | 动态参数更新 |
| 6 | `06_yaml_config` | Python | ★★☆☆☆ | YAML 参数文件 |
| 7 | `07_conditional_launch` | Python | ★★★☆☆ | 条件启动、GroupAction |
| 8 | `08_parameter_py` | Python | ★★☆☆☆ | Python 参数 |
| 9 | `09_launch_composition` | Python | ★★★☆☆ | 通过 Launch 组合组件 |
| 10 | `10_launch_events` | Python | ★★★★☆ | 事件驱动启动 |

### 模块 04：TF2 坐标变换（10 个练习）

| # | 练习 | 语言 | 难度 | 核心概念 |
|---|------|------|------|----------|
| 1 | `01_static_broadcaster` | C++ | ★★☆☆☆ | 静态变换广播器 |
| 2 | `02_dynamic_broadcaster` | C++ | ★★☆☆☆ | 动态变换广播器 |
| 3 | `03_tf_listener` | C++ | ★★☆☆☆ | 变换监听器、lookupTransform |
| 4 | `04_frame_chain` | C++ | ★★★☆☆ | 多帧链（map→odom→base→sensor） |
| 5 | `05_tf_time_travel` | C++ | ★★★☆☆ | 时间戳查询、tf2::TimePointZero |
| 6 | `06_tf_broadcaster_py` | Python | ★★☆☆☆ | Python 变换广播器 |
| 7 | `07_tf_listener_py` | Python | ★★☆☆☆ | Python lookupTransform |
| 8 | `08_coordinate_transform` | C++ | ★★★☆☆ | tf2::doTransform、跨帧点变换 |
| 9 | `09_tf_debug` | Python | ★★★☆☆ | TF 树分析、view_frames |
| 10 | `10_multi_robot_tf` | C++ | ★★★★☆ | 多机器人 TF、命名空间隔离 |

### 模块 05：URDF 机器人建模（8 个练习）

| # | 练习 | 语言 | 难度 | 核心概念 |
|---|------|------|------|----------|
| 1 | `01_first_urdf` | URDF | ★☆☆☆☆ | 最小 URDF、单连杆、robot 标签 |
| 2 | `02_links_and_joints` | URDF | ★★☆☆☆ | 多连杆、关节类型（旋转/固定） |
| 3 | `03_visual_geometry` | URDF | ★★☆☆☆ | 盒/柱/球几何体、材质 |
| 4 | `04_collision_inertia` | URDF | ★★★☆☆ | 碰撞几何、惯性属性 |
| 5 | `05_xacro_basics` | Xacro | ★★☆☆☆ | Xacro 命名空间、属性、宏 |
| 6 | `06_xacro_params` | Xacro | ★★★☆☆ | 参数化建模、可复用宏 |
| 7 | `07_robot_state_pub` | Python | ★★★☆☆ | Launch + robot_state_publisher |
| 8 | `08_urdf_tf_integration` | Python | ★★★★☆ | URDF-TF 集成、joint_states |

## 高效学习技巧

1. **不要赶进度** — 阅读每个练习的 `explain.md`，理解"为什么"，而不仅仅是"怎么做"。
2. **少用提示** — 先自己尝试 5-10 分钟再按 `h`。
3. **不要抄解答** — `solutions/` 目录只在你真正卡住 30 分钟以上时才使用。
4. **动手实验** — 解决一个练习后，试着修改它。换个 QoS 会怎样？再加一个订阅者呢？
5. **使用 ROS2 命令行工具** — 在做练习时，在另一个终端试试 `ros2 topic list`、`ros2 node info` 等命令。
6. **做笔记** — 记录你发现的规律（例如："发布者总是需要话题名 + QoS 深度"）。
7. **C++ 和 Python 练习对比着做** — 这有助于你理解同一概念在两种语言中的实现。

## 常见问题

**"ROS2 environment not found"（找不到 ROS2 环境）**
```bash
source /opt/ros/humble/setup.bash
```

**遇到看不懂的构建错误**
```bash
# 在监视模式下按 'e' 查看架构说明
# 按 'h' 查看提示
# 查看 test/ 目录下的测试文件，了解期望的行为
```

**想重新开始某个练习**
```bash
ros2lings reset <练习名称>
```

**想跳过当前练习**
在监视模式下按 `n`。

## 许可证

MIT — 详见 [LICENSE](LICENSE)。
