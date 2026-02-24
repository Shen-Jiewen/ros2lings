# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## [0.1.0] - 2026-02-24

### Added

- **CLI tool** — Rust-based interactive learning CLI with watch mode, hints, verify, reset, explain, and progress tracking
- **Module 01: Nodes & Topics** — 15 exercises covering node creation, publishers, subscribers, custom messages, QoS, lifecycle nodes, component nodes, namespaces, and multi-node processes (C++ & Python)
- **Module 02: Services & Actions** — 12 exercises covering service servers/clients, action servers/clients, custom `.srv`/`.action` definitions, introspection, and state machines (C++ & Python)
- **Module 03: Launch & Parameters** — 10 exercises covering launch files, multi-node launch, launch arguments, parameters, YAML config, conditional launch, composition, and event-driven launch (C++ & Python)
- **Module 04: TF2 Transforms** — 10 exercises covering static/dynamic broadcasters, transform listeners, frame chains, time travel queries, coordinate transforms, TF debugging, and multi-robot TF (C++ & Python)
- **Module 05: URDF & Robot Modeling** — 8 exercises covering URDF structure, links/joints, visual geometry, collision/inertia, Xacro basics/params, robot state publisher, and URDF-TF integration (URDF/Xacro & Python)
- **Shared interfaces package** (`ros2lings_interfaces`) with `AddTwoInts.srv` and `Fibonacci.action`
- **Solution files** for all 55 exercises
- **Per-exercise learning resources** — `explain.md` architecture explanations and 3 progressive hints per exercise
- **Automated testing** — GTest for C++ exercises, pytest for Python exercises, XML validation for URDF/Xacro
- **CI/CD pipeline** — GitHub Actions workflow for Cargo tests and ROS2 colcon build/test
- **Documentation** — Bilingual README (English + Chinese), CLAUDE.md development guide
