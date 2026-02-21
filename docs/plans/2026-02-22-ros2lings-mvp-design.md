# ROS2lings MVP 设计文档

**日期**: 2026-02-22
**范围**: MVP — CLI 工具 (Rust) + Module 01-02 (27 个练习)
**状态**: Approved

---

## 1. 概述

ROS2lings 是一个受 rustlings 启发的交互式命令行学习工具。学习者通过"写代码 → 编译 → 修复 → 理解"的循环掌握 ROS2 核心知识。

MVP 交付:
- Rust CLI 工具，支持 watch 模式、分级提示、进度追踪
- Module 01: Nodes & Topics (15 个练习)
- Module 02: Services & Actions (12 个练习)

## 2. 仓库结构

```
ros2lings/
├── Cargo.toml                    # Rust CLI 项目
├── Cargo.lock
├── info.toml                     # 所有练习的元数据定义
├── .ros2lings-state.txt          # 学习进度持久化 (gitignore)
│
├── src/                          # CLI 源码 (Rust)
│   ├── main.rs                   # 入口 + clap 命令定义
│   ├── app_state.rs              # 进度状态管理和持久化
│   ├── exercise.rs               # Exercise 结构体 + 编译/测试执行
│   ├── info_file.rs              # info.toml 解析
│   ├── watch.rs                  # 文件监控 + 事件循环
│   ├── verify.rs                 # 编译 → 测试 → 结果解析
│   ├── hint.rs                   # 分级提示显示
│   ├── explain.rs                # 架构说明渲染 (Markdown → terminal)
│   ├── output.rs                 # 终端输出格式化（颜色、进度条）
│   └── ros2_env.rs               # ROS2 环境检测
│
├── exercises/                    # 所有练习 (ROS2 packages)
│   ├── 01_nodes/
│   │   ├── 01_hello_node/
│   │   │   ├── src/hello_node.cpp
│   │   │   ├── test/test_hello_node.cpp
│   │   │   ├── hints/hint1.md, hint2.md, hint3.md
│   │   │   ├── explain.md
│   │   │   ├── CMakeLists.txt
│   │   │   └── package.xml
│   │   └── ...
│   └── 02_services/
│       └── ...
│
├── solutions/                    # 参考答案 (镜像 exercises/ 结构)
│   └── ...
│
└── docs/plans/
```

**关键决策**:
- 单仓库 (monorepo): CLI + 练习在同一个 Git 仓库
- 每个练习是一个独立的 ROS2 package
- info.toml 在仓库根目录集中管理元数据
- 进度文件 `.ros2lings-state.txt` 不提交到仓库
- 练习包名统一前缀 `ros2lings_` 避免冲突

## 3. CLI 命令

| 命令 | 行为 |
|---|---|
| `ros2lings` | 默认进入 watch 模式，监控当前练习文件变化，自动编译测试 |
| `ros2lings list` | 显示所有练习及状态: ✓ 完成 / ✗ 未完成 / ❯ 当前 |
| `ros2lings hint` | 显示当前练习的下一级提示 (递进: hint1 → hint2 → hint3) |
| `ros2lings verify [name]` | 手动验证指定练习 |
| `ros2lings reset <name>` | 从 solutions/ 恢复练习初始状态 |
| `ros2lings explain [name]` | 在终端渲染 explain.md |
| `ros2lings graph` | 显示学习进度条 + 模块完成状态概览 |

## 4. Watch 模式核心循环

```
notify watcher (exercises/ 目录)
  │
  ▼
检测文件变化 → 是否为当前练习？
  │ yes
  ▼
检查 "// I AM NOT DONE" 标记
  │ 已删除
  ▼
source /opt/ros/humble/setup.bash
  ▼
colcon build --packages-select <pkg>
  --build-base /tmp/ros2lings_build
  --install-base /tmp/ros2lings_install
  │
  ├─ 编译失败 → 解析错误 → 显示友好信息 → 等待下次变化
  │
  └─ 编译成功 → colcon test --packages-select <pkg>
                 │
                 ├─ 测试失败 → 显示失败详情
                 │
                 └─ 测试通过 → ✅ 练习完成! → 更新进度 → 自动跳转下一题
```

**实现要点**:
- 构建隔离: build/install 目录使用 /tmp，避免污染仓库
- ROS2 环境: 启动时检测 ROS_DISTRO，未 source 则自动处理
- 错误美化: 拦截 colcon/CMake/gcc 输出，解析后加颜色和说明
- 防抖: 文件变化事件 300ms 防抖

## 5. info.toml 元数据格式

```toml
format_version = 1

welcome_message = """
欢迎来到 ROS2lings！
你将通过修复"坏掉的" ROS2 程序来学习 ROS2 核心概念。
"""

final_message = """
恭喜！你已经完成了所有练习！
"""

[[exercises]]
name = "01_hello_node"
dir = "01_nodes/01_hello_node"
module = "Nodes & Topics"
mode = "fix"                    # fix | implement | explore | debug
language = "cpp"                # cpp | python | c
difficulty = 1                  # 1-5
estimated_minutes = 5
hint_count = 3
depends_on = []                 # 前置练习
test = true
hint = "看看 rclcpp::init() 需要什么参数。"
```

**与 rustlings 的差异**:
- 新增 module, mode, language, difficulty, estimated_minutes, hint_count 字段
- 新增 depends_on 支持练习前置依赖
- dir 是 ROS2 包目录路径 (非单文件)
- hint 用于快速提示, hints/ 目录提供分级详细提示

## 6. 练习内容

### Module 01: Nodes & Topics (15 个练习)

| # | 练习名 | 类型 | 语言 | 学习目标 | 时间 |
|---|---|---|---|---|---|
| 01 | hello_node | Fix | C++ | rclcpp::init/shutdown 生命周期 | 5min |
| 02 | first_publisher | Fix | C++ | Publisher 创建 + Timer 回调 | 10min |
| 03 | first_subscriber | Fix | C++ | Subscription 回调绑定 + QoS 基础 | 10min |
| 04 | pubsub_connect | Implement | C++ | 完整发布订阅系统 | 15min |
| 05 | custom_message | Implement | C++ | 定义 .msg 文件 + 使用 | 15min |
| 06 | multi_topic | Implement | C++ | 多 Topic 通信 + 消息过滤 | 20min |
| 07 | topic_introspection | Explore | C++ | ros2 topic CLI 检查话题 | 10min |
| 08 | node_lifecycle_basics | Explore | C++ | Node 构造函数内部 | 15min |
| 09 | hello_node_py | Fix | Python | rclpy 节点创建 | 5min |
| 10 | publisher_py | Fix | Python | rclpy Publisher + Timer | 10min |
| 11 | subscriber_py | Fix | Python | rclpy Subscription 回调 | 10min |
| 12 | qos_mismatch | Debug | C++ | QoS 不兼容诊断 | 15min |
| 13 | component_node | Implement | C++ | 组件化节点 (Composition) | 20min |
| 14 | namespace_remap | Fix | C++ | 命名空间和话题重映射 | 10min |
| 15 | multi_node_process | Implement | C++ | 单进程多节点 + Executor | 15min |

### Module 02: Services & Actions (12 个练习)

| # | 练习名 | 类型 | 语言 | 学习目标 | 时间 |
|---|---|---|---|---|---|
| 01 | first_service | Fix | C++ | Service Server 创建 + 回调 | 10min |
| 02 | service_client | Fix | C++ | 同步/异步 Client 调用 | 15min |
| 03 | service_pair | Implement | C++ | 完整 Server + Client 系统 | 20min |
| 04 | custom_srv | Implement | C++ | 定义 .srv 文件 + 使用 | 15min |
| 05 | service_py | Fix | Python | rclpy Service Server/Client | 10min |
| 06 | first_action_server | Fix | C++ | Action Server + Goal/Feedback/Result | 20min |
| 07 | action_client | Fix | C++ | Action Client + 反馈监听 | 20min |
| 08 | action_complete | Implement | C++ | 完整 Action 系统 (含 cancel) | 25min |
| 09 | custom_action | Implement | C++ | 定义 .action 文件 + 使用 | 15min |
| 10 | action_py | Fix | Python | rclpy Action Server/Client | 15min |
| 11 | service_introspection | Explore | C++ | ros2 service CLI 运行时检查 | 10min |
| 12 | action_state_machine | Debug | C++ | Action 状态机异常转换调试 | 20min |

## 7. 练习文件格式规范

### 源文件

```cpp
// I AM NOT DONE
//
// 练习: <name>
// 模块: <module>
// 难度: ★☆☆☆☆
//
// 学习目标:
//   <一句话描述>
//
// 说明:
//   <背景描述>
//
// 步骤:
//   1. <步骤>
//   2. <步骤>
//   3. 修复完成后，删除文件顶部的 "// I AM NOT DONE"
```

- `// I AM NOT DONE` 在文件顶部，删除后触发验证
- `// TODO: <描述>` 标记所有需修改的位置
- 保留足够的正确代码作为上下文
- 每个练习只引入 1-2 个新概念

### 提示文件

- `hints/hint1.md`: 方向性提示
- `hints/hint2.md`: 具体代码线索
- `hints/hint3.md`: 几乎是答案

### 测试文件

- 基于 gtest (C++) 或 pytest (Python)
- 只读，学习者不需修改
- 超时 30 秒
- Explore 类型用答案字符串校验

### CMakeLists.txt

- 包名: `ros2lings_<module>_<exercise>`
- 标准 ament_cmake 结构
- 测试用 ament_cmake_gtest / ament_cmake_pytest

## 8. Rust CLI 技术栈

| Crate | 用途 |
|---|---|
| clap 4.x | 命令行参数解析 (derive) |
| notify 8.x | 文件系统监控 (inotify) |
| crossterm 0.29+ | 终端 I/O, 颜色, raw mode |
| toml 0.9+ | info.toml 解析 |
| serde 1.x | 序列化/反序列化 |
| anyhow 1.x | 错误处理 |
| termimad 0.x | Markdown → 终端渲染 |

### 核心模块

- `main.rs` — clap CLI 入口
- `app_state.rs` — 进度状态管理 + .ros2lings-state.txt 持久化
- `exercise.rs` — Exercise 结构体 + colcon build/test 调用
- `info_file.rs` — info.toml 解析
- `watch.rs` — notify watcher + crossterm 事件循环
- `verify.rs` — 编译 → 测试 → 结果解析 pipeline
- `ros2_env.rs` — ROS2 环境检测和验证

### colcon 调用策略

- `colcon build --packages-select <pkg> --build-base /tmp/ros2lings_build --install-base /tmp/ros2lings_install`
- `colcon test --packages-select <pkg>` + 解析 gtest XML 结果
- 环境变量注入: 确保 ROS2 setup.bash 已 source

## 9. 测试策略

| 测试层 | 工具 | 覆盖 |
|---|---|---|
| CLI 单元测试 | cargo test | info.toml 解析, 进度管理, 输出格式化 |
| CLI 集成测试 | cargo test + mock | colcon 调用模拟, watch 事件 |
| 端到端测试 | 脚本 | 真实 ROS2 环境完整工作流 |
| 练习测试 | gtest / pytest | 每个练习的学习目标验证 |

## 10. 错误处理

- **编译失败**: 解析 stderr 提取关键错误行, 匹配常见模式给建议
- **测试失败**: 解析 gtest XML, 显示失败测试名 + 断言详情
- **环境问题**: 启动检测 ROS_DISTRO, colcon, cmake 等, 给出安装指引

## 11. 非功能需求

- 文件监控响应: 保存到开始编译 < 500ms
- 单练习编译: 增量编译 < 10s
- 测试执行: < 5s/练习
- 兼容: ROS2 Humble, Ubuntu 22.04 (native + WSL2)
- DDS: CycloneDDS (默认)
- 语言: MVP 阶段中文优先, CLI 输出英文
