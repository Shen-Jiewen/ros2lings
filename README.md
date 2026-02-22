# ROS2lings

[![CI](https://github.com/Shen-Jiewen/ros2lings/actions/workflows/ci.yml/badge.svg)](https://github.com/Shen-Jiewen/ros2lings/actions/workflows/ci.yml)

**Learn ROS2 by fixing broken code** — like [rustlings](https://github.com/rust-lang/rustlings), but for ROS2.

ROS2lings guides you through 55 exercises that teach ROS2 core concepts. Each exercise is a broken ROS2 package with `TODO` markers showing you what to fix. A CLI tool watches your edits, rebuilds, tests, and advances you automatically.

## Prerequisites

- **ROS2 Humble** (Ubuntu 22.04)
- **Rust** (stable toolchain)
- C++ build tools: `build-essential`, `cmake`
- Python 3.10+

## Installation

```bash
git clone https://github.com/Shen-Jiewen/ros2lings.git
cd ros2lings
cargo install --path .
```

## Usage

```bash
# Source ROS2
source /opt/ros/humble/setup.bash

# Start the interactive watch mode
ros2lings

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

### Module 04: TF2 Transforms (10 exercises)

| # | Exercise | Language | Concepts |
|---|----------|----------|----------|
| 1 | `01_static_broadcaster` | C++ | StaticTransformBroadcaster, static transform |
| 2 | `02_dynamic_broadcaster` | C++ | TransformBroadcaster, timer-based transform |
| 3 | `03_tf_listener` | C++ | TransformListener, Buffer, lookupTransform |
| 4 | `04_frame_chain` | C++ | Multi-frame chain (map→odom→base→sensor) |
| 5 | `05_tf_time_travel` | C++ | Timestamp queries, tf2::TimePointZero |
| 6 | `06_tf_broadcaster_py` | Python | Python TransformBroadcaster |
| 7 | `07_tf_listener_py` | Python | Python lookupTransform |
| 8 | `08_coordinate_transform` | C++ | tf2::doTransform, cross-frame point transform |
| 9 | `09_tf_debug` | Python | TF tree analysis, view_frames |
| 10 | `10_multi_robot_tf` | C++ | Multi-robot TF trees, namespace isolation |

### Module 05: URDF & Robot Modeling (8 exercises)

| # | Exercise | Language | Concepts |
|---|----------|----------|----------|
| 1 | `01_first_urdf` | URDF | Minimal URDF, single link, robot tag |
| 2 | `02_links_and_joints` | URDF | Multi-link, joint types (revolute/fixed) |
| 3 | `03_visual_geometry` | URDF | Box/cylinder/sphere geometry, materials |
| 4 | `04_collision_inertia` | URDF | Collision geometry, inertial properties |
| 5 | `05_xacro_basics` | Xacro | Xacro namespace, properties, macros |
| 6 | `06_xacro_params` | Xacro | Parameterized modeling, reusable macros |
| 7 | `07_robot_state_pub` | Python | Launch + robot_state_publisher |
| 8 | `08_urdf_tf_integration` | Python | URDF-TF integration, joint_states |

## How It Works

1. `ros2lings` presents you with the first unsolved exercise
2. Open the exercise source file and fix the `TODO` items
3. ROS2lings detects your changes, rebuilds with `colcon`, and runs tests
4. When tests pass, you automatically advance to the next exercise
5. Use `h` for hints, `l` to see the current exercise, `n` to skip

## License

MIT — see [LICENSE](LICENSE) for details.
