# ROS2lings

[![CI](https://github.com/Shen-Jiewen/ros2lings/actions/workflows/ci.yml/badge.svg)](https://github.com/Shen-Jiewen/ros2lings/actions/workflows/ci.yml)

**Learn ROS2 by fixing broken code** — like [rustlings](https://github.com/rust-lang/rustlings), but for ROS2.

[中文文档](README_CN.md)

## What is ROS2lings?

ROS2lings is an interactive, hands-on learning platform for ROS2. It provides **55 progressive exercises** across 5 modules that teach you ROS2 core concepts by having you fix broken code. Each exercise is a real ROS2 package with `TODO` markers guiding you on what to fix. A Rust-based CLI tool watches your edits, rebuilds, runs tests, and advances you automatically.

**Inspired by [rustlings](https://github.com/rust-lang/rustlings)** — if you've used rustlings to learn Rust, you'll feel right at home.

## How Learning Works

```
┌─────────────────────────────────────────────────────────┐
│  1. ros2lings presents you with a broken exercise       │
│  2. Read the comments, understand the TODO items        │
│  3. Fix the code in your editor                         │
│  4. ros2lings detects your changes automatically        │
│  5. It rebuilds with colcon and runs tests              │
│  6. Tests pass → you advance to the next exercise!      │
│  7. Stuck? Press 'h' for progressive hints              │
└─────────────────────────────────────────────────────────┘
```

Each exercise file looks like this:

```cpp
// I AM NOT DONE          ← Remove this line when you're done
//
// Exercise: hello_node
// Module: 01 - Nodes & Topics
// Difficulty: ★☆☆☆☆
//
// Steps:
//   1. Fix rclcpp::init() arguments
//   2. Fix node creation (hint: smart pointer needed)
//   3. Remove "// I AM NOT DONE" at the top

#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  // TODO: rclcpp::init() needs argc and argv
  rclcpp::init();

  // TODO: Node must be created with std::make_shared
  auto node = rclcpp::Node("hello_node");
  ...
}
```

## Prerequisites

| Requirement | Version |
|-------------|---------|
| **Ubuntu** | 22.04 LTS |
| **ROS2** | Humble Hawksbill |
| **Rust** | Stable toolchain |
| **C++ tools** | `build-essential`, `cmake` |
| **Python** | 3.10+ |

## Installation

```bash
# 1. Make sure ROS2 Humble is installed
#    See: https://docs.ros.org/en/humble/Installation.html

# 2. Install Rust (if not already installed)
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh

# 3. Clone the repository
git clone https://github.com/Shen-Jiewen/ros2lings.git
cd ros2lings

# 4. Build and install the CLI tool
cargo install --path .

# 5. Source the ROS2 environment
source /opt/ros/humble/setup.bash
```

## Quick Start

```bash
# Source ROS2 (do this every time you open a new terminal)
source /opt/ros/humble/setup.bash

# Start learning! This launches the interactive watch mode.
ros2lings
```

That's it! The CLI will present your first exercise. Open the file in your editor, fix the TODO items, save, and watch it auto-verify.

## CLI Commands

| Command | Description |
|---------|-------------|
| `ros2lings` | Start interactive watch mode (main learning loop) |
| `ros2lings list` | Show all exercises with completion status |
| `ros2lings hint` | Show next hint for the current exercise |
| `ros2lings verify` | Manually verify the current exercise |
| `ros2lings verify <name>` | Verify a specific exercise |
| `ros2lings reset <name>` | Reset an exercise to its original state |
| `ros2lings explain` | Show architecture explanation for current exercise |
| `ros2lings graph` | Show learning progress overview |

## Watch Mode Keyboard Shortcuts

| Key | Action |
|-----|--------|
| `q` | Quit |
| `h` | Show hint (press again for next hint, up to 3) |
| `e` | Show architecture explanation |
| `l` | Show current exercise info |
| `n` | Skip to next exercise |

## Step-by-Step Learning Guide

### Step 1: Understand the Structure

Each exercise is a standalone ROS2 package located in `exercises/`. They are organized by module:

```
exercises/
├── 01_nodes/          ← Nodes & Topics (start here!)
├── 02_services/       ← Services & Actions
├── 03_launch/         ← Launch & Parameters
├── 04_tf2/            ← TF2 Transforms
├── 05_urdf/           ← URDF & Robot Modeling
└── ros2lings_interfaces/  ← Shared message definitions
```

Each exercise package contains:

```
exercises/01_nodes/01_hello_node/
├── src/hello_node.cpp    ← The broken code YOU fix
├── test/                 ← Automated tests (validates your fix)
├── explain.md            ← Concept explanation with diagrams
├── hints/                ← 3 progressive hints
│   ├── hint1.md          ← Gentle nudge
│   ├── hint2.md          ← More specific guidance
│   └── hint3.md          ← Near-solution help
├── CMakeLists.txt        ← Build config (usually no changes needed)
└── package.xml           ← Package metadata
```

### Step 2: Start with Watch Mode

```bash
source /opt/ros/humble/setup.bash
ros2lings
```

The CLI shows you the first unsolved exercise and tells you which file to edit.

### Step 3: Read Before You Code

Before jumping into the code, read the exercise file from top to bottom. The comments contain:
- **Learning objectives** — what you'll understand after this exercise
- **Description** — context and background
- **Steps** — exactly what you need to do
- **TODO markers** — the specific lines to fix

Also read the `explain.md` file in the exercise directory for deeper conceptual understanding.

### Step 4: Fix the Code

Open the source file in your preferred editor (VS Code, vim, etc.) and fix each `TODO`. The exercise comments and hints guide you.

### Step 5: Use Hints Wisely

If you're stuck, press `h` in watch mode. Hints are progressive:
- **Hint 1**: A gentle nudge in the right direction
- **Hint 2**: More specific guidance
- **Hint 3**: Nearly tells you the answer

Try to solve it with as few hints as possible!

### Step 6: Remove the Done Marker

When all TODOs are fixed and you're confident in your solution, remove the `// I AM NOT DONE` line at the top of the file. This signals that you're done.

### Step 7: Watch the Auto-Verification

After saving, ros2lings will:
1. Build your exercise with `colcon build`
2. Run the automated tests with `colcon test`
3. If everything passes → you advance to the next exercise!
4. If something fails → you see the error and can fix it

### Step 8: Check Your Progress

```bash
ros2lings list     # See all exercises and your completion status
ros2lings graph    # Visual progress overview
```

### Step 9: Use Solutions as Reference

If you're completely stuck, reference solutions are in the `solutions/` directory with a parallel structure:

```
solutions/
├── 01_nodes/01_hello_node/src/hello_node.cpp
├── 01_nodes/02_first_publisher/src/first_publisher.cpp
└── ...
```

**Try to solve exercises on your own first!** Solutions are for learning from, not copying.

## Exercise Modes

Exercises come in 4 modes, each teaching differently:

| Mode | Count | Description |
|------|-------|-------------|
| **Fix** | 26 | Code has bugs — find and fix them |
| **Implement** | 20 | Partial code — complete the missing parts |
| **Explore** | 6 | Working code — modify to understand behavior |
| **Debug** | 3 | Subtle bugs — diagnose and repair |

## Recommended Learning Path

The exercises are designed to be done in order. Each module builds on the previous one.

| Week | Module | Exercises | What You'll Learn |
|------|--------|-----------|-------------------|
| 1-2 | **01: Nodes & Topics** | 15 exercises | Nodes, publishers, subscribers, custom messages, QoS, lifecycle, components |
| 3-4 | **02: Services & Actions** | 12 exercises | Service servers/clients, action servers/clients, custom interfaces |
| 5-6 | **03: Launch & Parameters** | 10 exercises | Launch files, parameters, YAML config, composition, events |
| 7-8 | **04: TF2 Transforms** | 10 exercises | Static/dynamic transforms, frame chains, multi-robot TF |
| 9-10 | **05: URDF & Robot Modeling** | 8 exercises | URDF, Xacro, robot state publisher, TF integration |

## Exercises

### Module 01: Nodes & Topics (15 exercises)

| # | Exercise | Lang | Difficulty | Concepts |
|---|----------|------|------------|----------|
| 1 | `01_hello_node` | C++ | ★☆☆☆☆ | Node creation, `rclcpp::init` |
| 2 | `02_first_publisher` | C++ | ★☆☆☆☆ | Publisher, timer callback |
| 3 | `03_first_subscriber` | C++ | ★☆☆☆☆ | Subscriber, message callback |
| 4 | `04_pubsub_connect` | C++ | ★★☆☆☆ | Pub/Sub wiring |
| 5 | `05_custom_message` | C++ | ★★☆☆☆ | Custom `.msg` definition |
| 6 | `06_multi_topic` | C++ | ★★☆☆☆ | Multiple publishers/subscribers |
| 7 | `07_topic_introspection` | C++ | ★★☆☆☆ | Topic listing, type inspection |
| 8 | `08_node_lifecycle_basics` | C++ | ★★★☆☆ | Lifecycle node states |
| 9 | `09_hello_node_py` | Python | ★☆☆☆☆ | Python node basics |
| 10 | `10_publisher_py` | Python | ★☆☆☆☆ | Python publisher |
| 11 | `11_subscriber_py` | Python | ★☆☆☆☆ | Python subscriber |
| 12 | `12_qos_mismatch` | C++ | ★★★☆☆ | QoS profile compatibility |
| 13 | `13_component_node` | C++ | ★★★☆☆ | Component nodes, composition |
| 14 | `14_namespace_remap` | C++ | ★★☆☆☆ | Namespaces, remapping |
| 15 | `15_multi_node_process` | C++ | ★★☆☆☆ | Multiple nodes in one process |

### Module 02: Services & Actions (12 exercises)

| # | Exercise | Lang | Difficulty | Concepts |
|---|----------|------|------------|----------|
| 1 | `01_first_service` | C++ | ★☆☆☆☆ | Service server |
| 2 | `02_service_client` | C++ | ★★☆☆☆ | Service client, async call |
| 3 | `03_service_pair` | C++ | ★★☆☆☆ | Client-server pair |
| 4 | `04_custom_srv` | C++ | ★★☆☆☆ | Custom `.srv` definition |
| 5 | `05_service_py` | Python | ★★☆☆☆ | Python service |
| 6 | `06_first_action_server` | C++ | ★★★☆☆ | Action server |
| 7 | `07_action_client` | C++ | ★★★☆☆ | Action client |
| 8 | `08_action_complete` | C++ | ★★★☆☆ | Full action workflow |
| 9 | `09_custom_action` | C++ | ★★★☆☆ | Custom `.action` definition |
| 10 | `10_action_py` | Python | ★★★☆☆ | Python action |
| 11 | `11_service_introspection` | C++ | ★★☆☆☆ | Service introspection |
| 12 | `12_action_state_machine` | C++ | ★★★★☆ | Action state management |

### Module 03: Launch & Parameters (10 exercises)

| # | Exercise | Lang | Difficulty | Concepts |
|---|----------|------|------------|----------|
| 1 | `01_first_launch` | Python | ★☆☆☆☆ | Launch file basics, Node action |
| 2 | `02_multi_node_launch` | Python | ★★☆☆☆ | Multi-node launch |
| 3 | `03_launch_arguments` | Python | ★★☆☆☆ | Launch arguments, configuration |
| 4 | `04_node_parameters` | C++ | ★★☆☆☆ | Parameter declaration and retrieval |
| 5 | `05_parameter_callback` | C++ | ★★★☆☆ | Dynamic parameter updates |
| 6 | `06_yaml_config` | Python | ★★☆☆☆ | YAML parameter files |
| 7 | `07_conditional_launch` | Python | ★★★☆☆ | Conditional launch, GroupAction |
| 8 | `08_parameter_py` | Python | ★★☆☆☆ | Python parameters |
| 9 | `09_launch_composition` | Python | ★★★☆☆ | Component composition via launch |
| 10 | `10_launch_events` | Python | ★★★★☆ | Event-driven launch |

### Module 04: TF2 Transforms (10 exercises)

| # | Exercise | Lang | Difficulty | Concepts |
|---|----------|------|------------|----------|
| 1 | `01_static_broadcaster` | C++ | ★★☆☆☆ | StaticTransformBroadcaster |
| 2 | `02_dynamic_broadcaster` | C++ | ★★☆☆☆ | TransformBroadcaster, timer-based |
| 3 | `03_tf_listener` | C++ | ★★☆☆☆ | TransformListener, Buffer, lookupTransform |
| 4 | `04_frame_chain` | C++ | ★★★☆☆ | Multi-frame chain (map→odom→base→sensor) |
| 5 | `05_tf_time_travel` | C++ | ★★★☆☆ | Timestamp queries, tf2::TimePointZero |
| 6 | `06_tf_broadcaster_py` | Python | ★★☆☆☆ | Python TransformBroadcaster |
| 7 | `07_tf_listener_py` | Python | ★★☆☆☆ | Python lookupTransform |
| 8 | `08_coordinate_transform` | C++ | ★★★☆☆ | tf2::doTransform, cross-frame point |
| 9 | `09_tf_debug` | Python | ★★★☆☆ | TF tree analysis, view_frames |
| 10 | `10_multi_robot_tf` | C++ | ★★★★☆ | Multi-robot TF, namespace isolation |

### Module 05: URDF & Robot Modeling (8 exercises)

| # | Exercise | Lang | Difficulty | Concepts |
|---|----------|------|------------|----------|
| 1 | `01_first_urdf` | URDF | ★☆☆☆☆ | Minimal URDF, single link, robot tag |
| 2 | `02_links_and_joints` | URDF | ★★☆☆☆ | Multi-link, joint types (revolute/fixed) |
| 3 | `03_visual_geometry` | URDF | ★★☆☆☆ | Box/cylinder/sphere geometry, materials |
| 4 | `04_collision_inertia` | URDF | ★★★☆☆ | Collision geometry, inertial properties |
| 5 | `05_xacro_basics` | Xacro | ★★☆☆☆ | Xacro namespace, properties, macros |
| 6 | `06_xacro_params` | Xacro | ★★★☆☆ | Parameterized modeling, reusable macros |
| 7 | `07_robot_state_pub` | Python | ★★★☆☆ | Launch + robot_state_publisher |
| 8 | `08_urdf_tf_integration` | Python | ★★★★☆ | URDF-TF integration, joint_states |

## Tips for Effective Learning

1. **Don't rush** — Read the `explain.md` for each exercise to understand the "why", not just the "how".
2. **Minimize hint usage** — Try 5-10 minutes on your own before pressing `h`.
3. **Don't copy solutions** — Use `solutions/` only after solving or if you're truly stuck for 30+ minutes.
4. **Experiment** — After solving an exercise, try modifying it. What happens if you change the QoS? Add another subscriber?
5. **Use ROS2 CLI tools** — While working on exercises, try `ros2 topic list`, `ros2 node info`, etc. in another terminal.
6. **Take notes** — Write down patterns you notice (e.g., "publishers always need a topic name + QoS depth").
7. **Do C++ and Python exercises back-to-back** — This helps you understand the same concepts in both languages.

## Troubleshooting

**"ROS2 environment not found"**
```bash
source /opt/ros/humble/setup.bash
```

**Build errors you don't understand**
```bash
# Press 'e' in watch mode to see the architecture explanation
# Press 'h' for hints
# Check the test file in test/ to understand what's expected
```

**Want to start over on an exercise**
```bash
ros2lings reset <exercise_name>
```

**Want to skip an exercise**
Press `n` in watch mode.

## License

MIT — see [LICENSE](LICENSE) for details.
