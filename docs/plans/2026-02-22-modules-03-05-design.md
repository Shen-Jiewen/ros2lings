# ROS2lings Modules 03-05 Design

## Overview

Add three new modules to ROS2lings, expanding from 27 to 55 exercises. All modules follow the existing independent-module pattern (each self-contained, no cross-module coupling).

**Target version**: v0.2.0
**New exercises**: 28 (10 + 10 + 8)
**Total exercises**: 55

## Decisions

- **Scope**: Intermediate core (Launch, TF2, URDF) — no Nav2/MoveIt2/Gazebo
- **Module order**: Launch → TF2 → URDF (each builds on prior concepts)
- **Language ratio**: ~70% C++ / 30% Python per module (matches existing style)
- **Exercises per module**: 8-12 (consistent with Modules 01/02)
- **Organization**: Independent modules (no shared "robot project" theme)
- **Exercise modes**: Mix of fix, implement, explore, debug (same four modes)

---

## Module 03: Launch & Parameters (10 exercises)

**Directory**: `exercises/03_launch/`
**New ROS2 deps**: `launch-ros`, `launch-testing`

| # | Name | Mode | Lang | Diff | Concept |
|---|------|------|------|------|---------|
| 28 | `01_first_launch` | fix | Python | 1 | Minimal launch file, Node action, single node startup |
| 29 | `02_multi_node_launch` | fix | Python | 1 | Launch multiple nodes from one file |
| 30 | `03_launch_arguments` | implement | Python | 2 | DeclareLaunchArgument, LaunchConfiguration |
| 31 | `04_node_parameters` | fix | C++ | 2 | declare_parameter, get_parameter, parameter types |
| 32 | `05_parameter_callback` | fix | C++ | 2 | on_set_parameters_callback, dynamic reconfigure |
| 33 | `06_yaml_config` | implement | Python | 2 | YAML parameter files, passing params via launch |
| 34 | `07_conditional_launch` | implement | Python | 3 | IfCondition, UnlessCondition, GroupAction |
| 35 | `08_parameter_py` | fix | Python | 2 | Python node parameter declaration and usage |
| 36 | `09_launch_composition` | implement | Python | 3 | ComposableNodeContainer, LoadComposableNode |
| 37 | `10_launch_events` | explore | Python | 3 | OnProcessExit, RegisterEventHandler, event-driven launch |

**Progression logic**:
- Exercises 01-03: Pure launch files (Python launch API basics)
- Exercises 04-06: Parameter system (C++ and Python, YAML config)
- Exercises 07-10: Advanced launch techniques (conditionals, composition, events)

**Testing approach**:
- Launch exercises: Test that nodes actually start and advertise expected topics/services
- Parameter exercises: Test parameter values via node API
- Use `launch_testing` for integration tests where appropriate

---

## Module 04: TF2 Transforms (10 exercises)

**Directory**: `exercises/04_tf2/`
**New ROS2 deps**: `tf2`, `tf2-ros`, `tf2-geometry-msgs`, `geometry-msgs`

| # | Name | Mode | Lang | Diff | Concept |
|---|------|------|------|------|---------|
| 38 | `01_static_broadcaster` | fix | C++ | 1 | StaticTransformBroadcaster, publish static transform |
| 39 | `02_dynamic_broadcaster` | fix | C++ | 2 | TransformBroadcaster, timer-based dynamic transform |
| 40 | `03_tf_listener` | fix | C++ | 2 | TransformListener, Buffer, lookupTransform |
| 41 | `04_frame_chain` | implement | C++ | 2 | Multi-frame chain (map→odom→base_link→sensor) |
| 42 | `05_tf_time_travel` | debug | C++ | 3 | Timestamp queries, tf2::TimePointZero, history |
| 43 | `06_tf_broadcaster_py` | fix | Python | 1 | Python TransformBroadcaster |
| 44 | `07_tf_listener_py` | fix | Python | 2 | Python lookupTransform |
| 45 | `08_coordinate_transform` | implement | C++ | 3 | do_transform_point, cross-frame point transform |
| 46 | `09_tf_debug` | explore | C++ | 2 | tf2_tools view_frames, TF tree analysis |
| 47 | `10_multi_robot_tf` | implement | C++ | 3 | Multi-robot TF trees, namespace isolation |

**Progression logic**:
- Exercises 01-03: Core TF2 operations (static broadcast, dynamic broadcast, listen)
- Exercises 04-05: Deep understanding (frame chains, time-based queries)
- Exercises 06-07: Python coverage of core operations
- Exercises 08-10: Real-world applications (coordinate math, debugging, multi-robot)

**Testing approach**:
- Broadcaster tests: Verify transforms appear on `/tf` or `/tf_static` topics
- Listener tests: Create known transforms, verify lookupTransform returns correct values
- Use test fixtures that publish known transform trees

---

## Module 05: URDF & Robot Modeling (8 exercises)

**Directory**: `exercises/05_urdf/`
**New ROS2 deps**: `urdf`, `robot-state-publisher`, `joint-state-publisher`, `xacro`

| # | Name | Mode | Lang | Diff | Concept |
|---|------|------|------|------|---------|
| 48 | `01_first_urdf` | fix | C++ | 1 | Minimal URDF (single link), robot_state_publisher |
| 49 | `02_links_and_joints` | fix | C++ | 2 | Multi-link + joint, joint types (revolute/fixed) |
| 50 | `03_visual_geometry` | implement | C++ | 2 | Mesh/cylinder/box geometry, material colors |
| 51 | `04_collision_inertia` | implement | C++ | 2 | Collision geometry, inertial properties (mass, inertia matrix) |
| 52 | `05_xacro_basics` | fix | C++ | 2 | Xacro macros, properties, include, URDF generation |
| 53 | `06_xacro_params` | implement | C++ | 3 | Parameterized modeling (configurable arm length/joints) |
| 54 | `07_robot_state_pub` | implement | Python | 2 | Launch + robot_state_publisher + joint_state_publisher |
| 55 | `08_urdf_tf_integration` | debug | C++ | 3 | URDF → TF tree integrity, joint_states publishing |

**Progression logic**:
- Exercises 01-02: URDF syntax fundamentals
- Exercises 03-04: Visual, collision, and physical properties
- Exercises 05-06: Xacro automation and parameterization
- Exercises 07-08: Launch integration and TF2 cross-module debugging

**Testing approach**:
- URDF exercises: Parse URDF/Xacro, validate link/joint counts and properties
- State publisher tests: Verify expected TF frames are published
- Integration test: Full URDF → TF pipeline validation

---

## CI Impact

### Updated `ros2-tests` job dependencies

New apt packages to add:
```
ros-humble-launch-ros
ros-humble-launch-testing
ros-humble-launch-testing-ament-cmake
ros-humble-tf2
ros-humble-tf2-ros
ros-humble-tf2-geometry-msgs
ros-humble-geometry-msgs
ros-humble-urdf
ros-humble-robot-state-publisher
ros-humble-joint-state-publisher
ros-humble-xacro
```

### Updated colcon build paths
```bash
colcon build --paths \
  exercises/01_nodes/* \
  exercises/02_services/* \
  exercises/03_launch/* \
  exercises/04_tf2/* \
  exercises/05_urdf/*
```

### Updated solution overlay
The existing overlay script auto-discovers `solutions/*/` so adding `solutions/03_launch/`, `solutions/04_tf2/`, `solutions/05_urdf/` needs no CI changes.

---

## File Structure Per Exercise (unchanged)

```
exercises/0X_module/NN_exercise_name/
  CMakeLists.txt
  package.xml
  explain.md
  hints/
    hint1.md
    hint2.md
    hint3.md
  src/
    exercise_name.cpp (or .py)
  test/
    test_exercise_name.cpp (or .py)
  [msg/ | srv/ | action/ | urdf/ | launch/]  # as needed
```

Solutions follow the same minimal pattern: `solutions/0X_module/NN_exercise_name/src/` only.

---

## Implementation Order

Implement one module at a time, fully (exercises + solutions + tests + info.toml entries), then push and verify CI:

1. Module 03: Launch & Parameters
2. Module 04: TF2 Transforms
3. Module 05: URDF & Robot Modeling
4. Update README exercise table
5. Update final_message in info.toml
6. Tag v0.2.0
