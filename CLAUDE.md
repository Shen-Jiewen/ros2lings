# CLAUDE.md — ROS2lings Development Guide

## Project Overview

ROS2lings is a rustlings-style interactive learning platform for ROS2. Students fix broken ROS2 packages guided by TODO markers, with a Rust CLI that watches edits, rebuilds, tests, and advances progress automatically.

**Tech stack:** Rust (CLI), C++/Python (exercises), CMake/colcon (build), GTest/pytest (tests)

## Repository Structure

```
.
├── src/                  # Rust CLI source code
├── exercises/            # 55 exercise packages (broken code for students)
│   ├── 01_nodes/         # Module 01: Nodes & Topics (15 exercises)
│   ├── 02_services/      # Module 02: Services & Actions (12 exercises)
│   ├── 03_launch/        # Module 03: Launch & Parameters (10 exercises)
│   ├── 04_tf2/           # Module 04: TF2 Transforms (10 exercises)
│   ├── 05_urdf/          # Module 05: URDF & Robot Modeling (8 exercises)
│   └── ros2lings_interfaces/  # Shared msg/srv/action definitions
├── solutions/            # Complete working solutions (parallel structure)
├── info.toml             # Exercise metadata (order, difficulty, hints, deps)
├── Cargo.toml            # Rust project config
└── .github/workflows/    # CI pipeline
```

## Build & Test Commands

```bash
# CLI tool
cargo build                    # Build CLI
cargo test                     # Run CLI unit tests
cargo fmt                      # Format Rust code
cargo clippy                   # Lint Rust code

# ROS2 exercises (requires: source /opt/ros/humble/setup.bash)
colcon build                   # Build all exercise packages
colcon test                    # Run all exercise tests
colcon test-result --verbose   # Show test results
colcon build --packages-select <package_name>   # Build single package
colcon test --packages-select <package_name>    # Test single package
```

## Exercise Package Structure

Each exercise follows this layout:

```
exercises/<module>/<exercise_name>/
├── package.xml           # ROS2 package metadata
├── CMakeLists.txt        # Build configuration
├── explain.md            # Concept explanation with diagrams
├── hints/
│   ├── hint1.md          # Progressive hints (3 levels)
│   ├── hint2.md
│   └── hint3.md
├── src/                  # Student source files (with TODO markers)
│   └── *.cpp or *.py
└── test/                 # Automated tests
    └── test_*.cpp or test_*.py
```

## Development Conventions

### Commit Messages

Follow conventional commits format:

```
<type>(<scope>): <description>

Types: feat, fix, docs, style, refactor, test, chore, ci
Scope: cli, module01-05, ci, docs (optional)
```

Examples:
- `feat(module03): add exercise 28 — first_launch`
- `fix: use MultiThreadedExecutor for action cancel test`
- `docs: update README with bilingual guide`

### Exercise Source Files

Every exercise source file must include:

1. **Done marker** at the top: `// I AM NOT DONE` (C++), `# I AM NOT DONE` (Python), `<!-- I AM NOT DONE -->` (URDF/XML)
2. **Header comment block** with: exercise name, module, difficulty (stars), learning objectives, description, steps
3. **TODO markers** at each location students need to fix
4. **ROS2LINGS_TEST guard** for C++ files with `main()`:

```cpp
#ifndef ROS2LINGS_TEST
int main(int argc, char * argv[]) { ... }
#endif
```

### C++ Test Pattern

Tests include the student source directly and compile it in the test binary:

```cpp
#define ROS2LINGS_TEST    // Excludes student's main()
#include "../src/student_file.cpp"
#include <gtest/gtest.h>

TEST(ExerciseName, TestCase) { ... }
```

### Python Test Pattern

Python tests import or exec the student script and validate behavior using pytest.

### info.toml Entry

Each exercise must have a corresponding entry in `info.toml`:

```toml
[[exercises]]
name = "exercise_name"
dir = "module/exercise_name"
module = "Module Name"
mode = "fix"              # fix | implement | explore | debug
language = "cpp"          # cpp | python | urdf | xacro
difficulty = 1            # 1-5
estimated_minutes = 10
hint_count = 3
test = true
depends_on = ["previous_exercise"]
hint = "Inline hint text in Chinese"
```

### Adding a New Exercise

1. Create the package directory under `exercises/<module>/`
2. Write the broken source file with TODO markers and done marker
3. Write the test file (GTest for C++, pytest for Python)
4. Write `explain.md` with concept explanation
5. Write 3 hint files in `hints/`
6. Add `CMakeLists.txt` and `package.xml`
7. Add the exercise entry to `info.toml`
8. Add the solution in `solutions/<module>/`
9. Verify: build passes with solution overlaid, tests pass

### Code Style

- **Rust:** Follow `rustfmt` defaults, all clippy warnings clean
- **C++:** ROS2 style — `snake_case` for functions/variables, `CamelCase` for classes/types
- **Python:** PEP 8, use `rclpy` conventions
- **URDF/Xacro:** Proper XML indentation (2 spaces)

## CI Pipeline

GitHub Actions runs on push/PR to main:

1. **Cargo checks** — `cargo fmt --check`, `cargo clippy`, `cargo test`
2. **ROS2 build** — Overlay solutions onto exercises, `colcon build`, `colcon test`

All tests must pass before merging.

## Key Design Decisions

- **Exercises are real ROS2 packages** — each can be built and tested independently with colcon
- **Tests validate student code** — C++ tests `#include` student source with `ROS2LINGS_TEST` guard to exclude `main()`
- **Progressive hints** — 3 levels per exercise, from gentle nudge to near-solution
- **State persistence** — `.ros2lings-state.txt` tracks user progress (gitignored)
- **Solution overlay in CI** — CI copies solution files over exercise files before building, so CI validates that solutions actually pass tests
