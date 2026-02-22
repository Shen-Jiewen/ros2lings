# ROS2lings

[![CI](https://github.com/USERNAME/ros2lings/actions/workflows/ci.yml/badge.svg)](https://github.com/USERNAME/ros2lings/actions/workflows/ci.yml)

**Learn ROS2 by fixing broken code** — like [rustlings](https://github.com/rust-lang/rustlings), but for ROS2.

ROS2lings guides you through 27 exercises that teach ROS2 core concepts. Each exercise is a broken ROS2 package with `TODO` markers showing you what to fix. A CLI tool watches your edits, rebuilds, tests, and advances you automatically.

## Prerequisites

- **ROS2 Humble** (Ubuntu 22.04)
- **Rust** (stable toolchain)
- C++ build tools: `build-essential`, `cmake`
- Python 3.10+

## Installation

```bash
git clone https://github.com/USERNAME/ros2lings.git
cd ros2lings
cargo install --path .
```

## Usage

```bash
# Source ROS2
source /opt/ros/humble/setup.bash

# Start the interactive watch mode
ros2lings watch

# Other commands
ros2lings list          # Show all exercises
ros2lings hint          # Get a hint for the current exercise
ros2lings verify        # Manually verify the current exercise
ros2lings reset <name>  # Reset an exercise to its original state
```

## Exercises

### Module 01: Nodes & Topics (15 exercises)

| # | Exercise | Language | Concepts |
|---|----------|----------|----------|
| 1 | `01_hello_node` | C++ | Node creation, `rclcpp::init` |
| 2 | `02_first_publisher` | C++ | Publisher, timer callback |
| 3 | `03_first_subscriber` | C++ | Subscriber, message callback |
| 4 | `04_pubsub_connect` | C++ | Pub/Sub wiring |
| 5 | `05_custom_message` | C++ | Custom `.msg` definition |
| 6 | `06_multi_topic` | C++ | Multiple publishers/subscribers |
| 7 | `07_topic_introspection` | C++ | Topic listing, type inspection |
| 8 | `08_node_lifecycle_basics` | C++ | Lifecycle node states |
| 9 | `09_hello_node_py` | Python | Python node basics |
| 10 | `10_publisher_py` | Python | Python publisher |
| 11 | `11_subscriber_py` | Python | Python subscriber |
| 12 | `12_qos_mismatch` | C++ | QoS profile compatibility |
| 13 | `13_component_node` | C++ | Component nodes, composition |
| 14 | `14_namespace_remap` | C++ | Namespaces, remapping |
| 15 | `15_multi_node_process` | C++ | Multiple nodes in one process |

### Module 02: Services & Actions (12 exercises)

| # | Exercise | Language | Concepts |
|---|----------|----------|----------|
| 1 | `01_first_service` | C++ | Service server |
| 2 | `02_service_client` | C++ | Service client, async call |
| 3 | `03_service_pair` | C++ | Client-server pair |
| 4 | `04_custom_srv` | C++ | Custom `.srv` definition |
| 5 | `05_service_py` | Python | Python service |
| 6 | `06_first_action_server` | C++ | Action server |
| 7 | `07_action_client` | C++ | Action client |
| 8 | `08_action_complete` | C++ | Full action workflow |
| 9 | `09_custom_action` | C++ | Custom `.action` definition |
| 10 | `10_action_py` | Python | Python action |
| 11 | `11_service_introspection` | C++ | Service introspection |
| 12 | `12_action_state_machine` | C++ | Action state management |

## How It Works

1. `ros2lings watch` presents you with the first unsolved exercise
2. Open the exercise source file and fix the `TODO` items
3. ROS2lings detects your changes, rebuilds with `colcon`, and runs tests
4. When tests pass, you automatically advance to the next exercise
5. Use `h` for hints, `l` to see the current exercise, `n` to skip

## License

MIT — see [LICENSE](LICENSE) for details.
