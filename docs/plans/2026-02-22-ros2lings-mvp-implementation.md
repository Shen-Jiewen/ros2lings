# ROS2lings MVP Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Build a rustlings-inspired CLI tool (Rust) that teaches ROS2 through fix-the-code exercises, with Module 01 (Nodes & Topics, 15 exercises) and Module 02 (Services & Actions, 12 exercises).

**Architecture:** Rust CLI using clap for commands, notify for filesystem watching, crossterm for terminal output. Each exercise is an independent ROS2 package compiled/tested via colcon. Progress tracked in a local state file.

**Tech Stack:** Rust (clap, notify, crossterm, toml, serde, anyhow, termimad), ROS2 Humble (colcon, ament_cmake, gtest, pytest), C++/Python exercises.

---

## Phase 1: Project Scaffolding

### Task 1: Initialize Cargo project with dependencies

**Files:**
- Create: `Cargo.toml`
- Create: `src/main.rs`
- Create: `.gitignore`

**Step 1: Create Cargo project**

```bash
cd /home/javen/ros2_ws
cargo init --name ros2lings
```

**Step 2: Set up Cargo.toml with all dependencies**

Replace `Cargo.toml` with:

```toml
[package]
name = "ros2lings"
version = "0.1.0"
edition = "2021"
description = "Learn ROS2 by fixing broken code — like rustlings, but for ROS2"
license = "MIT"

[dependencies]
anyhow = "1"
clap = { version = "4", features = ["derive"] }
crossterm = "0.28"
notify = "8"
notify-debouncer-mini = "0.5"
serde = { version = "1", features = ["derive"] }
toml = "0.8"
termimad = "0.30"

[dev-dependencies]
tempfile = "3"
assert_cmd = "2"
predicates = "3"
```

**Step 3: Set up .gitignore**

```
/target
.ros2lings-state.txt
/tmp/ros2lings_build
/tmp/ros2lings_install
```

**Step 4: Write minimal main.rs**

```rust
fn main() {
    println!("ROS2lings — Learn ROS2 by fixing code!");
}
```

**Step 5: Verify it compiles**

```bash
cd /home/javen/ros2_ws && cargo build
```
Expected: BUILD SUCCESS

**Step 6: Commit**

```bash
git add Cargo.toml Cargo.lock src/main.rs .gitignore
git commit -m "chore: initialize ros2lings cargo project with dependencies"
```

---

### Task 2: CLI entry point with clap subcommands

**Files:**
- Modify: `src/main.rs`
- Create: `src/cmd.rs`

**Step 1: Write test for CLI parsing**

Add to end of `src/main.rs`:

```rust
#[cfg(test)]
mod tests {
    use super::*;
    use clap::Parser;

    #[test]
    fn test_no_subcommand_is_watch_mode() {
        let cli = Cli::try_parse_from(["ros2lings"]).unwrap();
        assert!(cli.command.is_none());
    }

    #[test]
    fn test_list_subcommand() {
        let cli = Cli::try_parse_from(["ros2lings", "list"]).unwrap();
        assert!(matches!(cli.command, Some(Commands::List)));
    }

    #[test]
    fn test_hint_subcommand() {
        let cli = Cli::try_parse_from(["ros2lings", "hint"]).unwrap();
        assert!(matches!(cli.command, Some(Commands::Hint)));
    }

    #[test]
    fn test_verify_with_name() {
        let cli = Cli::try_parse_from(["ros2lings", "verify", "hello_node"]).unwrap();
        if let Some(Commands::Verify { name }) = cli.command {
            assert_eq!(name, Some("hello_node".to_string()));
        } else {
            panic!("Expected Verify command");
        }
    }

    #[test]
    fn test_reset_requires_name() {
        let result = Cli::try_parse_from(["ros2lings", "reset"]);
        assert!(result.is_err());
    }

    #[test]
    fn test_reset_with_name() {
        let cli = Cli::try_parse_from(["ros2lings", "reset", "hello_node"]).unwrap();
        if let Some(Commands::Reset { name }) = cli.command {
            assert_eq!(name, "hello_node");
        } else {
            panic!("Expected Reset command");
        }
    }

    #[test]
    fn test_explain_subcommand() {
        let cli = Cli::try_parse_from(["ros2lings", "explain"]).unwrap();
        if let Some(Commands::Explain { name }) = cli.command {
            assert!(name.is_none());
        } else {
            panic!("Expected Explain command");
        }
    }
}
```

**Step 2: Run test to verify it fails**

```bash
cargo test
```
Expected: FAIL — `Cli` and `Commands` not defined

**Step 3: Implement CLI struct**

Replace all non-test code in `src/main.rs`:

```rust
mod cmd;

use clap::{Parser, Subcommand};

#[derive(Parser)]
#[command(
    name = "ros2lings",
    version,
    about = "Learn ROS2 by fixing broken code!",
    long_about = "ROS2lings — like rustlings, but for ROS2.\n\n\
        Fix broken ROS2 programs, learn how things work under the hood.\n\
        Run without arguments to enter watch mode."
)]
pub struct Cli {
    #[command(subcommand)]
    pub command: Option<Commands>,
}

#[derive(Subcommand)]
pub enum Commands {
    /// Show all exercises and their completion status
    List,
    /// Show a hint for the current exercise (progressive: hint1 → hint2 → hint3)
    Hint,
    /// Manually verify an exercise (defaults to current)
    Verify {
        /// Exercise name (defaults to current exercise)
        name: Option<String>,
    },
    /// Reset an exercise to its initial state
    Reset {
        /// Exercise name (required)
        name: String,
    },
    /// Show the architectural explanation for an exercise
    Explain {
        /// Exercise name (defaults to current exercise)
        name: Option<String>,
    },
    /// Show learning progress overview
    Graph,
}

fn main() -> anyhow::Result<()> {
    let cli = Cli::parse();
    cmd::run(cli)
}
```

Create `src/cmd.rs`:

```rust
use crate::{Cli, Commands};

pub fn run(cli: Cli) -> anyhow::Result<()> {
    match cli.command {
        None => {
            println!("Entering watch mode... (not yet implemented)");
            Ok(())
        }
        Some(cmd) => match cmd {
            Commands::List => {
                println!("Listing exercises... (not yet implemented)");
                Ok(())
            }
            Commands::Hint => {
                println!("Showing hint... (not yet implemented)");
                Ok(())
            }
            Commands::Verify { name } => {
                println!("Verifying {:?}... (not yet implemented)", name);
                Ok(())
            }
            Commands::Reset { name } => {
                println!("Resetting {}... (not yet implemented)", name);
                Ok(())
            }
            Commands::Explain { name } => {
                println!("Explaining {:?}... (not yet implemented)", name);
                Ok(())
            }
            Commands::Graph => {
                println!("Showing progress... (not yet implemented)");
                Ok(())
            }
        },
    }
}
```

**Step 4: Run tests to verify they pass**

```bash
cargo test
```
Expected: All 7 tests PASS

**Step 5: Verify CLI runs**

```bash
cargo run
cargo run -- list
cargo run -- --help
```

**Step 6: Commit**

```bash
git add src/main.rs src/cmd.rs
git commit -m "feat: add CLI entry point with clap subcommands"
```

---

## Phase 2: Core Data Layer

### Task 3: info.toml parsing

**Files:**
- Create: `src/info_file.rs`
- Create: `info.toml` (minimal, 2 exercises for testing)
- Modify: `src/main.rs` (add `mod info_file`)

**Step 1: Write tests for info.toml parsing**

Create `src/info_file.rs`:

```rust
use anyhow::{Context, Result};
use serde::Deserialize;
use std::path::Path;

#[derive(Deserialize)]
pub struct InfoFile {
    pub format_version: u32,
    pub welcome_message: Option<String>,
    pub final_message: Option<String>,
    pub exercises: Vec<ExerciseInfo>,
}

#[derive(Deserialize, Clone, Debug)]
pub struct ExerciseInfo {
    pub name: String,
    pub dir: String,
    pub module: String,
    pub mode: ExerciseMode,
    pub language: Language,
    pub difficulty: u8,
    pub estimated_minutes: u32,
    pub hint_count: u32,
    #[serde(default)]
    pub depends_on: Vec<String>,
    #[serde(default = "default_true")]
    pub test: bool,
    #[serde(default)]
    pub hint: String,
}

fn default_true() -> bool {
    true
}

#[derive(Deserialize, Clone, Debug, PartialEq)]
#[serde(rename_all = "lowercase")]
pub enum ExerciseMode {
    Fix,
    Implement,
    Explore,
    Debug,
}

#[derive(Deserialize, Clone, Debug, PartialEq)]
#[serde(rename_all = "lowercase")]
pub enum Language {
    Cpp,
    Python,
    C,
}

impl InfoFile {
    pub fn parse(path: &Path) -> Result<Self> {
        let content = std::fs::read_to_string(path)
            .with_context(|| format!("Failed to read info.toml at {}", path.display()))?;
        let info: InfoFile = toml::from_str(&content)
            .with_context(|| "Failed to parse info.toml")?;

        if info.format_version != 1 {
            anyhow::bail!(
                "Unsupported info.toml format_version: {}. Expected 1.",
                info.format_version
            );
        }

        if info.exercises.is_empty() {
            anyhow::bail!("info.toml contains no exercises");
        }

        Ok(info)
    }
}

impl ExerciseInfo {
    /// Returns the package name used by colcon (ros2lings_<name>)
    pub fn package_name(&self) -> String {
        format!("ros2lings_{}", self.name)
    }

    /// Returns the path to the exercise directory relative to exercises/
    pub fn exercise_path(&self, exercises_root: &Path) -> std::path::PathBuf {
        exercises_root.join(&self.dir)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Write;
    use tempfile::NamedTempFile;

    fn write_temp_toml(content: &str) -> NamedTempFile {
        let mut f = NamedTempFile::new().unwrap();
        f.write_all(content.as_bytes()).unwrap();
        f
    }

    #[test]
    fn test_parse_valid_info_toml() {
        let toml = r#"
format_version = 1
welcome_message = "Welcome!"
final_message = "Done!"

[[exercises]]
name = "01_hello_node"
dir = "01_nodes/01_hello_node"
module = "Nodes & Topics"
mode = "fix"
language = "cpp"
difficulty = 1
estimated_minutes = 5
hint_count = 3
test = true
hint = "Check rclcpp::init() args."
"#;
        let f = write_temp_toml(toml);
        let info = InfoFile::parse(f.path()).unwrap();
        assert_eq!(info.exercises.len(), 1);
        assert_eq!(info.exercises[0].name, "01_hello_node");
        assert_eq!(info.exercises[0].mode, ExerciseMode::Fix);
        assert_eq!(info.exercises[0].language, Language::Cpp);
        assert_eq!(info.exercises[0].difficulty, 1);
    }

    #[test]
    fn test_parse_multiple_exercises() {
        let toml = r#"
format_version = 1

[[exercises]]
name = "01_hello_node"
dir = "01_nodes/01_hello_node"
module = "Nodes & Topics"
mode = "fix"
language = "cpp"
difficulty = 1
estimated_minutes = 5
hint_count = 3
hint = "hint"

[[exercises]]
name = "02_first_publisher"
dir = "01_nodes/02_first_publisher"
module = "Nodes & Topics"
mode = "fix"
language = "cpp"
difficulty = 1
estimated_minutes = 10
hint_count = 3
depends_on = ["01_hello_node"]
hint = "hint"
"#;
        let f = write_temp_toml(toml);
        let info = InfoFile::parse(f.path()).unwrap();
        assert_eq!(info.exercises.len(), 2);
        assert_eq!(
            info.exercises[1].depends_on,
            vec!["01_hello_node".to_string()]
        );
    }

    #[test]
    fn test_reject_wrong_format_version() {
        let toml = r#"
format_version = 99

[[exercises]]
name = "test"
dir = "test"
module = "test"
mode = "fix"
language = "cpp"
difficulty = 1
estimated_minutes = 5
hint_count = 1
hint = "hint"
"#;
        let f = write_temp_toml(toml);
        let result = InfoFile::parse(f.path());
        assert!(result.is_err());
        assert!(result
            .unwrap_err()
            .to_string()
            .contains("Unsupported info.toml format_version"));
    }

    #[test]
    fn test_reject_empty_exercises() {
        let toml = r#"
format_version = 1
"#;
        let f = write_temp_toml(toml);
        let result = InfoFile::parse(f.path());
        assert!(result.is_err());
    }

    #[test]
    fn test_package_name() {
        let info = ExerciseInfo {
            name: "01_hello_node".to_string(),
            dir: "01_nodes/01_hello_node".to_string(),
            module: "Nodes & Topics".to_string(),
            mode: ExerciseMode::Fix,
            language: Language::Cpp,
            difficulty: 1,
            estimated_minutes: 5,
            hint_count: 3,
            depends_on: vec![],
            test: true,
            hint: String::new(),
        };
        assert_eq!(info.package_name(), "ros2lings_01_hello_node");
    }

    #[test]
    fn test_default_test_is_true() {
        let toml = r#"
format_version = 1

[[exercises]]
name = "test"
dir = "test"
module = "test"
mode = "fix"
language = "cpp"
difficulty = 1
estimated_minutes = 5
hint_count = 1
hint = "hint"
"#;
        let f = write_temp_toml(toml);
        let info = InfoFile::parse(f.path()).unwrap();
        assert!(info.exercises[0].test);
    }

    #[test]
    fn test_all_modes_parse() {
        for mode in ["fix", "implement", "explore", "debug"] {
            let toml = format!(
                r#"
format_version = 1
[[exercises]]
name = "t"
dir = "t"
module = "t"
mode = "{mode}"
language = "cpp"
difficulty = 1
estimated_minutes = 1
hint_count = 1
hint = "h"
"#
            );
            let f = write_temp_toml(&toml);
            assert!(InfoFile::parse(f.path()).is_ok(), "Failed for mode: {mode}");
        }
    }

    #[test]
    fn test_all_languages_parse() {
        for lang in ["cpp", "python", "c"] {
            let toml = format!(
                r#"
format_version = 1
[[exercises]]
name = "t"
dir = "t"
module = "t"
mode = "fix"
language = "{lang}"
difficulty = 1
estimated_minutes = 1
hint_count = 1
hint = "h"
"#
            );
            let f = write_temp_toml(&toml);
            assert!(
                InfoFile::parse(f.path()).is_ok(),
                "Failed for language: {lang}"
            );
        }
    }
}
```

**Step 2: Add module to main.rs**

Add `mod info_file;` to the top of `src/main.rs` (after `mod cmd;`).

**Step 3: Run tests to verify they pass**

```bash
cargo test info_file
```
Expected: All 8 tests PASS

**Step 4: Commit**

```bash
git add src/info_file.rs src/main.rs
git commit -m "feat: add info.toml parsing with ExerciseInfo struct"
```

---

### Task 4: Exercise struct with done-marker detection

**Files:**
- Create: `src/exercise.rs`
- Modify: `src/main.rs` (add `mod exercise`)

**Step 1: Write tests**

Create `src/exercise.rs`:

```rust
use crate::info_file::ExerciseInfo;
use anyhow::{Context, Result};
use std::path::{Path, PathBuf};

const DONE_MARKER: &str = "// I AM NOT DONE";
const DONE_MARKER_PY: &str = "# I AM NOT DONE";

pub struct Exercise {
    pub info: ExerciseInfo,
    pub exercises_root: PathBuf,
}

impl Exercise {
    pub fn new(info: ExerciseInfo, exercises_root: PathBuf) -> Self {
        Self {
            info,
            exercises_root,
        }
    }

    /// Full path to the exercise directory
    pub fn dir_path(&self) -> PathBuf {
        self.exercises_root.join(&self.info.dir)
    }

    /// Find the main source file in the exercise
    pub fn source_file(&self) -> Result<PathBuf> {
        let dir = self.dir_path();
        let src_dir = dir.join("src");

        if src_dir.is_dir() {
            // Look for .cpp, .py, or .c files in src/
            for entry in std::fs::read_dir(&src_dir)
                .with_context(|| format!("Failed to read {}", src_dir.display()))?
            {
                let entry = entry?;
                let path = entry.path();
                if let Some(ext) = path.extension() {
                    match ext.to_str() {
                        Some("cpp" | "py" | "c") => return Ok(path),
                        _ => {}
                    }
                }
            }
        }

        // For Python packages, check for .py directly in the exercise dir
        for entry in std::fs::read_dir(&dir)
            .with_context(|| format!("Failed to read {}", dir.display()))?
        {
            let entry = entry?;
            let path = entry.path();
            if let Some(ext) = path.extension() {
                if ext == "py" {
                    return Ok(path);
                }
            }
        }

        anyhow::bail!(
            "No source file found in exercise '{}'",
            self.info.name
        )
    }

    /// Check if the exercise still has the "I AM NOT DONE" marker
    pub fn has_done_marker(source_path: &Path) -> Result<bool> {
        let content = std::fs::read_to_string(source_path)
            .with_context(|| format!("Failed to read {}", source_path.display()))?;

        Ok(content.contains(DONE_MARKER) || content.contains(DONE_MARKER_PY))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::info_file::{ExerciseMode, Language};
    use std::fs;
    use tempfile::TempDir;

    fn make_test_info(name: &str) -> ExerciseInfo {
        ExerciseInfo {
            name: name.to_string(),
            dir: format!("01_nodes/{name}"),
            module: "Nodes & Topics".to_string(),
            mode: ExerciseMode::Fix,
            language: Language::Cpp,
            difficulty: 1,
            estimated_minutes: 5,
            hint_count: 3,
            depends_on: vec![],
            test: true,
            hint: String::new(),
        }
    }

    #[test]
    fn test_dir_path() {
        let info = make_test_info("hello_node");
        let ex = Exercise::new(info, PathBuf::from("/tmp/exercises"));
        assert_eq!(
            ex.dir_path(),
            PathBuf::from("/tmp/exercises/01_nodes/hello_node")
        );
    }

    #[test]
    fn test_source_file_finds_cpp() {
        let tmp = TempDir::new().unwrap();
        let src_dir = tmp.path().join("01_nodes/hello/src");
        fs::create_dir_all(&src_dir).unwrap();
        fs::write(src_dir.join("hello.cpp"), "// code").unwrap();

        let info = ExerciseInfo {
            name: "hello".to_string(),
            dir: "01_nodes/hello".to_string(),
            module: "test".to_string(),
            mode: ExerciseMode::Fix,
            language: Language::Cpp,
            difficulty: 1,
            estimated_minutes: 5,
            hint_count: 1,
            depends_on: vec![],
            test: true,
            hint: String::new(),
        };
        let ex = Exercise::new(info, tmp.path().to_path_buf());
        let source = ex.source_file().unwrap();
        assert!(source.to_str().unwrap().ends_with("hello.cpp"));
    }

    #[test]
    fn test_has_done_marker_cpp() {
        let tmp = TempDir::new().unwrap();
        let file = tmp.path().join("test.cpp");
        fs::write(&file, "// I AM NOT DONE\nint main() {}").unwrap();
        assert!(Exercise::has_done_marker(&file).unwrap());
    }

    #[test]
    fn test_has_done_marker_python() {
        let tmp = TempDir::new().unwrap();
        let file = tmp.path().join("test.py");
        fs::write(&file, "# I AM NOT DONE\nimport rclpy").unwrap();
        assert!(Exercise::has_done_marker(&file).unwrap());
    }

    #[test]
    fn test_no_done_marker() {
        let tmp = TempDir::new().unwrap();
        let file = tmp.path().join("test.cpp");
        fs::write(&file, "int main() { return 0; }").unwrap();
        assert!(!Exercise::has_done_marker(&file).unwrap());
    }
}
```

**Step 2: Add module to main.rs**

Add `mod exercise;` to `src/main.rs`.

**Step 3: Run tests**

```bash
cargo test exercise
```
Expected: All 5 tests PASS

**Step 4: Commit**

```bash
git add src/exercise.rs src/main.rs
git commit -m "feat: add Exercise struct with done-marker detection"
```

---

### Task 5: AppState — progress persistence

**Files:**
- Create: `src/app_state.rs`
- Modify: `src/main.rs` (add `mod app_state`)

**Step 1: Write tests and implementation**

Create `src/app_state.rs`:

```rust
use crate::info_file::ExerciseInfo;
use anyhow::{Context, Result};
use std::collections::HashSet;
use std::path::{Path, PathBuf};

const STATE_FILE: &str = ".ros2lings-state.txt";

pub struct AppState {
    pub exercises: Vec<ExerciseInfo>,
    pub current_index: usize,
    pub done: HashSet<String>,
    state_file: PathBuf,
}

impl AppState {
    /// Load or create state from the project root
    pub fn load(project_root: &Path, exercises: Vec<ExerciseInfo>) -> Result<Self> {
        let state_file = project_root.join(STATE_FILE);
        let mut state = AppState {
            exercises,
            current_index: 0,
            done: HashSet::new(),
            state_file,
        };

        if state.state_file.exists() {
            state.read_state()?;
        }

        Ok(state)
    }

    fn read_state(&mut self) -> Result<()> {
        let content = std::fs::read_to_string(&self.state_file)
            .with_context(|| "Failed to read state file")?;

        for line in content.lines() {
            let line = line.trim();
            if line.is_empty() || line.starts_with('#') {
                continue;
            }
            if let Some(value) = line.strip_prefix("current=") {
                // Find index of exercise with this name
                if let Some(idx) = self
                    .exercises
                    .iter()
                    .position(|e| e.name == value)
                {
                    self.current_index = idx;
                }
            } else if let Some(value) = line.strip_prefix("done=") {
                for name in value.split(',') {
                    let name = name.trim();
                    if !name.is_empty() {
                        self.done.insert(name.to_string());
                    }
                }
            }
        }

        Ok(())
    }

    pub fn save(&self) -> Result<()> {
        let done_list: Vec<&str> = self.done.iter().map(|s| s.as_str()).collect();
        let current_name = &self.exercises[self.current_index].name;
        let content = format!(
            "# ROS2lings learning progress — do not edit manually\ncurrent={}\ndone={}\n",
            current_name,
            done_list.join(",")
        );
        std::fs::write(&self.state_file, content)
            .with_context(|| "Failed to write state file")?;
        Ok(())
    }

    /// Mark current exercise as done and advance to the next incomplete one
    pub fn done_current_exercise(&mut self) -> Result<()> {
        let name = self.exercises[self.current_index].name.clone();
        self.done.insert(name);

        // Find next undone exercise
        for i in 0..self.exercises.len() {
            let idx = (self.current_index + 1 + i) % self.exercises.len();
            if !self.done.contains(&self.exercises[idx].name) {
                self.current_index = idx;
                self.save()?;
                return Ok(());
            }
        }

        // All exercises done
        self.save()?;
        Ok(())
    }

    pub fn current_exercise(&self) -> &ExerciseInfo {
        &self.exercises[self.current_index]
    }

    pub fn is_done(&self, name: &str) -> bool {
        self.done.contains(name)
    }

    pub fn all_done(&self) -> bool {
        self.done.len() >= self.exercises.len()
    }

    pub fn progress(&self) -> (usize, usize) {
        (self.done.len(), self.exercises.len())
    }

    /// Find exercise index by name, returns None if not found
    pub fn find_exercise(&self, name: &str) -> Option<usize> {
        self.exercises.iter().position(|e| e.name == name)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::info_file::{ExerciseMode, Language};
    use tempfile::TempDir;

    fn make_exercises() -> Vec<ExerciseInfo> {
        vec![
            ExerciseInfo {
                name: "ex1".to_string(),
                dir: "m01/ex1".to_string(),
                module: "M1".to_string(),
                mode: ExerciseMode::Fix,
                language: Language::Cpp,
                difficulty: 1,
                estimated_minutes: 5,
                hint_count: 1,
                depends_on: vec![],
                test: true,
                hint: "h".to_string(),
            },
            ExerciseInfo {
                name: "ex2".to_string(),
                dir: "m01/ex2".to_string(),
                module: "M1".to_string(),
                mode: ExerciseMode::Fix,
                language: Language::Cpp,
                difficulty: 1,
                estimated_minutes: 5,
                hint_count: 1,
                depends_on: vec![],
                test: true,
                hint: "h".to_string(),
            },
            ExerciseInfo {
                name: "ex3".to_string(),
                dir: "m01/ex3".to_string(),
                module: "M1".to_string(),
                mode: ExerciseMode::Fix,
                language: Language::Cpp,
                difficulty: 1,
                estimated_minutes: 5,
                hint_count: 1,
                depends_on: vec![],
                test: true,
                hint: "h".to_string(),
            },
        ]
    }

    #[test]
    fn test_new_state_starts_at_first_exercise() {
        let tmp = TempDir::new().unwrap();
        let state = AppState::load(tmp.path(), make_exercises()).unwrap();
        assert_eq!(state.current_index, 0);
        assert_eq!(state.current_exercise().name, "ex1");
        assert!(state.done.is_empty());
    }

    #[test]
    fn test_save_and_reload() {
        let tmp = TempDir::new().unwrap();
        {
            let mut state = AppState::load(tmp.path(), make_exercises()).unwrap();
            state.done.insert("ex1".to_string());
            state.current_index = 1;
            state.save().unwrap();
        }
        {
            let state = AppState::load(tmp.path(), make_exercises()).unwrap();
            assert_eq!(state.current_index, 1);
            assert!(state.is_done("ex1"));
            assert!(!state.is_done("ex2"));
        }
    }

    #[test]
    fn test_done_current_advances() {
        let tmp = TempDir::new().unwrap();
        let mut state = AppState::load(tmp.path(), make_exercises()).unwrap();
        assert_eq!(state.current_exercise().name, "ex1");

        state.done_current_exercise().unwrap();
        assert_eq!(state.current_exercise().name, "ex2");
        assert!(state.is_done("ex1"));
    }

    #[test]
    fn test_done_skips_completed() {
        let tmp = TempDir::new().unwrap();
        let mut state = AppState::load(tmp.path(), make_exercises()).unwrap();
        state.done.insert("ex2".to_string());

        state.done_current_exercise().unwrap();
        // Should skip ex2 (already done) and go to ex3
        assert_eq!(state.current_exercise().name, "ex3");
    }

    #[test]
    fn test_all_done() {
        let tmp = TempDir::new().unwrap();
        let mut state = AppState::load(tmp.path(), make_exercises()).unwrap();
        assert!(!state.all_done());
        state.done.insert("ex1".to_string());
        state.done.insert("ex2".to_string());
        state.done.insert("ex3".to_string());
        assert!(state.all_done());
    }

    #[test]
    fn test_progress() {
        let tmp = TempDir::new().unwrap();
        let mut state = AppState::load(tmp.path(), make_exercises()).unwrap();
        assert_eq!(state.progress(), (0, 3));
        state.done.insert("ex1".to_string());
        assert_eq!(state.progress(), (1, 3));
    }

    #[test]
    fn test_find_exercise() {
        let tmp = TempDir::new().unwrap();
        let state = AppState::load(tmp.path(), make_exercises()).unwrap();
        assert_eq!(state.find_exercise("ex2"), Some(1));
        assert_eq!(state.find_exercise("nonexistent"), None);
    }
}
```

**Step 2: Add module to main.rs**

Add `mod app_state;` to `src/main.rs`.

**Step 3: Run tests**

```bash
cargo test app_state
```
Expected: All 7 tests PASS

**Step 4: Commit**

```bash
git add src/app_state.rs src/main.rs
git commit -m "feat: add AppState for progress tracking and persistence"
```

---

## Phase 3: ROS2 Integration

### Task 6: ROS2 environment detection

**Files:**
- Create: `src/ros2_env.rs`
- Modify: `src/main.rs` (add `mod ros2_env`)

**Step 1: Write tests and implementation**

Create `src/ros2_env.rs`:

```rust
use anyhow::{Context, Result};
use std::path::PathBuf;
use std::process::Command;

pub struct Ros2Env {
    pub distro: String,
    pub setup_bash: PathBuf,
}

impl Ros2Env {
    /// Detect ROS2 environment. Checks ROS_DISTRO env var and common paths.
    pub fn detect() -> Result<Self> {
        // Check if ROS_DISTRO is set (environment already sourced)
        if let Ok(distro) = std::env::var("ROS_DISTRO") {
            let setup = PathBuf::from(format!("/opt/ros/{distro}/setup.bash"));
            if setup.exists() {
                return Ok(Self {
                    distro,
                    setup_bash: setup,
                });
            }
        }

        // Try common ROS2 installation paths
        for distro in &["humble", "jazzy", "iron"] {
            let setup = PathBuf::from(format!("/opt/ros/{distro}/setup.bash"));
            if setup.exists() {
                return Ok(Self {
                    distro: distro.to_string(),
                    setup_bash: setup,
                });
            }
        }

        anyhow::bail!(
            "ROS2 not found. Please install ROS2 Humble and source setup.bash:\n  \
             sudo apt install ros-humble-desktop\n  \
             source /opt/ros/humble/setup.bash"
        )
    }

    /// Check that required tools are available
    pub fn check_dependencies(&self) -> Result<Vec<String>> {
        let mut missing = Vec::new();

        for tool in &["colcon", "cmake"] {
            if Command::new("which")
                .arg(tool)
                .output()
                .map(|o| !o.status.success())
                .unwrap_or(true)
            {
                missing.push(tool.to_string());
            }
        }

        Ok(missing)
    }

    /// Build a shell command prefix that sources ROS2 setup.bash first
    pub fn shell_prefix(&self) -> String {
        format!("source {} && ", self.setup_bash.display())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_detect_finds_ros2() {
        // This test will pass in CI/dev environments with ROS2 installed
        // and skip gracefully if ROS2 is not found
        match Ros2Env::detect() {
            Ok(env) => {
                assert!(!env.distro.is_empty());
                assert!(env.setup_bash.exists());
            }
            Err(_) => {
                eprintln!("SKIP: ROS2 not installed");
            }
        }
    }

    #[test]
    fn test_shell_prefix_format() {
        let env = Ros2Env {
            distro: "humble".to_string(),
            setup_bash: PathBuf::from("/opt/ros/humble/setup.bash"),
        };
        assert_eq!(
            env.shell_prefix(),
            "source /opt/ros/humble/setup.bash && "
        );
    }
}
```

**Step 2: Add module to main.rs**

Add `mod ros2_env;` to `src/main.rs`.

**Step 3: Run tests**

```bash
cargo test ros2_env
```
Expected: All 2 tests PASS (or 1 SKIP + 1 PASS if ROS2 not sourced)

**Step 4: Commit**

```bash
git add src/ros2_env.rs src/main.rs
git commit -m "feat: add ROS2 environment detection"
```

---

### Task 7: Verify pipeline — colcon build and test

**Files:**
- Create: `src/verify.rs`
- Modify: `src/main.rs` (add `mod verify`)

**Step 1: Write tests and implementation**

Create `src/verify.rs`:

```rust
use crate::exercise::Exercise;
use crate::ros2_env::Ros2Env;
use anyhow::{Context, Result};
use std::path::{Path, PathBuf};
use std::process::{Command, Output};

#[derive(Debug, PartialEq)]
pub enum VerifyResult {
    /// Exercise still has "I AM NOT DONE" marker
    NotReady,
    /// colcon build failed
    BuildFailed(String),
    /// colcon test failed
    TestFailed(String),
    /// All passed
    Success,
}

pub struct VerifyPipeline {
    ros2_env: Ros2Env,
    project_root: PathBuf,
}

impl VerifyPipeline {
    pub fn new(ros2_env: Ros2Env, project_root: PathBuf) -> Self {
        Self {
            ros2_env,
            project_root,
        }
    }

    /// Run the full verify pipeline for an exercise
    pub fn verify(&self, exercise: &Exercise) -> Result<VerifyResult> {
        // Step 1: Check done marker
        let source = exercise.source_file()?;
        if Exercise::has_done_marker(&source)? {
            return Ok(VerifyResult::NotReady);
        }

        // Step 2: Build
        let build_result = self.colcon_build(exercise)?;
        if !build_result.status.success() {
            let stderr = String::from_utf8_lossy(&build_result.stderr).to_string();
            let stdout = String::from_utf8_lossy(&build_result.stdout).to_string();
            return Ok(VerifyResult::BuildFailed(format!(
                "{}\n{}",
                stdout, stderr
            )));
        }

        // Step 3: Test (if exercise has tests)
        if exercise.info.test {
            let test_result = self.colcon_test(exercise)?;
            if !test_result.status.success() {
                let stderr = String::from_utf8_lossy(&test_result.stderr).to_string();
                let stdout = String::from_utf8_lossy(&test_result.stdout).to_string();
                return Ok(VerifyResult::TestFailed(format!(
                    "{}\n{}",
                    stdout, stderr
                )));
            }

            // Also check test results
            let test_result_output = self.colcon_test_result(exercise)?;
            if !test_result_output.status.success() {
                let stdout =
                    String::from_utf8_lossy(&test_result_output.stdout).to_string();
                return Ok(VerifyResult::TestFailed(stdout));
            }
        }

        Ok(VerifyResult::Success)
    }

    fn colcon_build(&self, exercise: &Exercise) -> Result<Output> {
        let pkg = exercise.info.package_name();
        let exercises_dir = self.project_root.join("exercises");
        let shell_cmd = format!(
            "{}colcon build \
             --paths {} \
             --packages-select {} \
             --build-base /tmp/ros2lings_build \
             --install-base /tmp/ros2lings_install \
             --event-handlers console_direct+ \
             --cmake-args -DCMAKE_BUILD_TYPE=Debug",
            self.ros2_env.shell_prefix(),
            exercise.dir_path().display(),
            pkg,
        );

        Command::new("bash")
            .args(["-c", &shell_cmd])
            .current_dir(&exercises_dir)
            .output()
            .with_context(|| format!("Failed to run colcon build for {}", pkg))
    }

    fn colcon_test(&self, exercise: &Exercise) -> Result<Output> {
        let pkg = exercise.info.package_name();
        let exercises_dir = self.project_root.join("exercises");
        let shell_cmd = format!(
            "{}colcon test \
             --packages-select {} \
             --build-base /tmp/ros2lings_build \
             --install-base /tmp/ros2lings_install \
             --event-handlers console_direct+",
            self.ros2_env.shell_prefix(),
            pkg,
        );

        Command::new("bash")
            .args(["-c", &shell_cmd])
            .current_dir(&exercises_dir)
            .output()
            .with_context(|| format!("Failed to run colcon test for {}", pkg))
    }

    fn colcon_test_result(&self, exercise: &Exercise) -> Result<Output> {
        let pkg = exercise.info.package_name();
        let shell_cmd = format!(
            "{}colcon test-result \
             --build-base /tmp/ros2lings_build",
            self.ros2_env.shell_prefix(),
        );

        Command::new("bash")
            .args(["-c", &shell_cmd])
            .output()
            .with_context(|| format!("Failed to get test results for {}", pkg))
    }
}

/// Extract the most useful error information from colcon/cmake/gcc output
pub fn format_build_error(raw_output: &str) -> String {
    let mut errors = Vec::new();

    for line in raw_output.lines() {
        // GCC/G++ errors
        if line.contains(": error:") || line.contains(": fatal error:") {
            errors.push(line.trim().to_string());
        }
        // CMake errors
        else if line.starts_with("CMake Error") {
            errors.push(line.trim().to_string());
        }
    }

    if errors.is_empty() {
        // Return last 20 lines as fallback
        raw_output
            .lines()
            .rev()
            .take(20)
            .collect::<Vec<_>>()
            .into_iter()
            .rev()
            .collect::<Vec<_>>()
            .join("\n")
    } else {
        errors.join("\n")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_format_build_error_gcc() {
        let output = r#"
Starting >>> ros2lings_01_hello_node
--- stderr: ros2lings_01_hello_node
/path/hello_node.cpp:10:5: error: 'rclcpp' has not been declared
   10 |     rclcpp::init();
      |     ^~~~~~
---
Failed   <<< ros2lings_01_hello_node
"#;
        let formatted = format_build_error(output);
        assert!(formatted.contains("error:"));
        assert!(formatted.contains("rclcpp"));
    }

    #[test]
    fn test_format_build_error_cmake() {
        let output = "CMake Error at CMakeLists.txt:5:\n  Could not find rclcpp\n";
        let formatted = format_build_error(output);
        assert!(formatted.contains("CMake Error"));
    }

    #[test]
    fn test_format_build_error_fallback() {
        let output = "some random output\nwith multiple lines\n";
        let formatted = format_build_error(output);
        assert!(formatted.contains("some random output"));
    }
}
```

**Step 2: Add module to main.rs**

Add `mod verify;` to `src/main.rs`.

**Step 3: Run tests**

```bash
cargo test verify
```
Expected: All 3 tests PASS

**Step 4: Commit**

```bash
git add src/verify.rs src/main.rs
git commit -m "feat: add verify pipeline for colcon build and test"
```

---

## Phase 4: Terminal Output & CLI Commands

### Task 8: Terminal output formatting

**Files:**
- Create: `src/output.rs`
- Modify: `src/main.rs` (add `mod output`)

**Step 1: Write output helpers**

Create `src/output.rs`:

```rust
use crossterm::style::{Attribute, Color, ResetColor, SetAttribute, SetForegroundColor};
use std::io::{self, Write};

pub fn print_success(msg: &str) {
    let mut stdout = io::stdout();
    let _ = write!(
        stdout,
        "{}{}  {} {}{}",
        SetForegroundColor(Color::Green),
        SetAttribute(Attribute::Bold),
        "✓",
        msg,
        ResetColor
    );
    let _ = writeln!(stdout);
}

pub fn print_error(msg: &str) {
    let mut stderr = io::stderr();
    let _ = write!(
        stderr,
        "{}{}  {} {}{}",
        SetForegroundColor(Color::Red),
        SetAttribute(Attribute::Bold),
        "✗",
        msg,
        ResetColor
    );
    let _ = writeln!(stderr);
}

pub fn print_warning(msg: &str) {
    let mut stdout = io::stdout();
    let _ = write!(
        stdout,
        "{}{}  {} {}{}",
        SetForegroundColor(Color::Yellow),
        SetAttribute(Attribute::Bold),
        "⚠",
        msg,
        ResetColor
    );
    let _ = writeln!(stdout);
}

pub fn print_info(msg: &str) {
    let mut stdout = io::stdout();
    let _ = write!(
        stdout,
        "{}  ℹ {}{}",
        SetForegroundColor(Color::Cyan),
        msg,
        ResetColor
    );
    let _ = writeln!(stdout);
}

pub fn print_exercise_header(name: &str, module: &str, mode: &str, difficulty: u8) {
    let stars = "★".repeat(difficulty as usize)
        + &"☆".repeat(5 - difficulty as usize);
    println!();
    let mut stdout = io::stdout();
    let _ = write!(
        stdout,
        "{}{}── {} ({}) ──{}",
        SetForegroundColor(Color::Blue),
        SetAttribute(Attribute::Bold),
        name,
        module,
        ResetColor,
    );
    let _ = writeln!(stdout);
    println!("   Type: {}  Difficulty: {}", mode, stars);
    println!();
}

pub fn print_progress_bar(done: usize, total: usize) {
    let width = 40;
    let filled = if total > 0 {
        (done * width) / total
    } else {
        0
    };
    let empty = width - filled;

    let mut stdout = io::stdout();
    let _ = write!(
        stdout,
        "  Progress: [{}{}{}{}] {}/{} ({:.0}%)\n",
        SetForegroundColor(Color::Green),
        "█".repeat(filled),
        SetForegroundColor(Color::DarkGrey),
        "░".repeat(empty),
        done,
        total,
        if total > 0 {
            (done as f64 / total as f64) * 100.0
        } else {
            0.0
        }
    );
    let _ = write!(stdout, "{}", ResetColor);
}

pub fn print_welcome(message: &str) {
    let mut stdout = io::stdout();
    let _ = writeln!(
        stdout,
        "{}{}{}{}",
        SetForegroundColor(Color::Cyan),
        SetAttribute(Attribute::Bold),
        message,
        ResetColor,
    );
}
```

**Step 2: Add module to main.rs**

Add `mod output;` to `src/main.rs`.

**Step 3: Verify it compiles**

```bash
cargo build
```
Expected: BUILD SUCCESS

**Step 4: Commit**

```bash
git add src/output.rs src/main.rs
git commit -m "feat: add terminal output formatting with colors"
```

---

### Task 9: Wire up CLI commands (list, hint, verify, reset, explain, graph)

**Files:**
- Modify: `src/cmd.rs` (full rewrite with real logic)
- Create: `src/hint.rs`
- Create: `src/explain.rs`
- Modify: `src/main.rs`

**Step 1: Create hint module**

Create `src/hint.rs`:

```rust
use crate::info_file::ExerciseInfo;
use anyhow::{Context, Result};
use std::path::Path;

/// Show the next unseen hint for an exercise.
/// Returns (hint_level, hint_content).
pub fn show_hint(
    exercise: &ExerciseInfo,
    exercises_root: &Path,
    hint_level: u32,
) -> Result<(u32, String)> {
    // First try the quick hint from info.toml
    if hint_level == 0 && !exercise.hint.is_empty() {
        return Ok((0, exercise.hint.clone()));
    }

    // Then try hints/hint{N}.md files
    let level = if hint_level == 0 { 1 } else { hint_level };
    let hint_file = exercises_root
        .join(&exercise.dir)
        .join("hints")
        .join(format!("hint{}.md", level));

    if hint_file.exists() {
        let content = std::fs::read_to_string(&hint_file)
            .with_context(|| format!("Failed to read {}", hint_file.display()))?;
        Ok((level, content))
    } else if level <= exercise.hint_count {
        anyhow::bail!("Hint file not found: {}", hint_file.display());
    } else {
        Ok((level, "No more hints available. You've seen all hints for this exercise!".to_string()))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::info_file::{ExerciseMode, Language};
    use std::fs;
    use tempfile::TempDir;

    #[test]
    fn test_show_quick_hint() {
        let info = ExerciseInfo {
            name: "test".to_string(),
            dir: "m/test".to_string(),
            module: "M".to_string(),
            mode: ExerciseMode::Fix,
            language: Language::Cpp,
            difficulty: 1,
            estimated_minutes: 5,
            hint_count: 1,
            depends_on: vec![],
            test: true,
            hint: "Quick hint here".to_string(),
        };
        let tmp = TempDir::new().unwrap();
        let (level, content) = show_hint(&info, tmp.path(), 0).unwrap();
        assert_eq!(level, 0);
        assert_eq!(content, "Quick hint here");
    }

    #[test]
    fn test_show_file_hint() {
        let tmp = TempDir::new().unwrap();
        let hints_dir = tmp.path().join("m/test/hints");
        fs::create_dir_all(&hints_dir).unwrap();
        fs::write(hints_dir.join("hint1.md"), "# Hint 1\nDetailed hint.").unwrap();

        let info = ExerciseInfo {
            name: "test".to_string(),
            dir: "m/test".to_string(),
            module: "M".to_string(),
            mode: ExerciseMode::Fix,
            language: Language::Cpp,
            difficulty: 1,
            estimated_minutes: 5,
            hint_count: 2,
            depends_on: vec![],
            test: true,
            hint: "quick".to_string(),
        };
        let (level, content) = show_hint(&info, tmp.path(), 1).unwrap();
        assert_eq!(level, 1);
        assert!(content.contains("Detailed hint"));
    }
}
```

**Step 2: Create explain module**

Create `src/explain.rs`:

```rust
use crate::info_file::ExerciseInfo;
use anyhow::{Context, Result};
use std::path::Path;

/// Read and return the explain.md content for an exercise
pub fn get_explanation(
    exercise: &ExerciseInfo,
    exercises_root: &Path,
) -> Result<String> {
    let explain_file = exercises_root
        .join(&exercise.dir)
        .join("explain.md");

    if !explain_file.exists() {
        anyhow::bail!(
            "No explanation available for '{}'. File not found: {}",
            exercise.name,
            explain_file.display()
        );
    }

    std::fs::read_to_string(&explain_file)
        .with_context(|| format!("Failed to read {}", explain_file.display()))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::info_file::{ExerciseMode, Language};
    use std::fs;
    use tempfile::TempDir;

    #[test]
    fn test_get_explanation() {
        let tmp = TempDir::new().unwrap();
        let ex_dir = tmp.path().join("m/test");
        fs::create_dir_all(&ex_dir).unwrap();
        fs::write(ex_dir.join("explain.md"), "# Node\nExplanation here.").unwrap();

        let info = ExerciseInfo {
            name: "test".to_string(),
            dir: "m/test".to_string(),
            module: "M".to_string(),
            mode: ExerciseMode::Fix,
            language: Language::Cpp,
            difficulty: 1,
            estimated_minutes: 5,
            hint_count: 1,
            depends_on: vec![],
            test: true,
            hint: String::new(),
        };
        let content = get_explanation(&info, tmp.path()).unwrap();
        assert!(content.contains("Explanation here"));
    }

    #[test]
    fn test_missing_explanation() {
        let tmp = TempDir::new().unwrap();
        let info = ExerciseInfo {
            name: "test".to_string(),
            dir: "m/test".to_string(),
            module: "M".to_string(),
            mode: ExerciseMode::Fix,
            language: Language::Cpp,
            difficulty: 1,
            estimated_minutes: 5,
            hint_count: 1,
            depends_on: vec![],
            test: true,
            hint: String::new(),
        };
        assert!(get_explanation(&info, tmp.path()).is_err());
    }
}
```

**Step 3: Rewrite cmd.rs with real logic**

Replace `src/cmd.rs`:

```rust
use crate::app_state::AppState;
use crate::exercise::Exercise;
use crate::info_file::InfoFile;
use crate::output;
use crate::ros2_env::Ros2Env;
use crate::verify::{self, VerifyPipeline, VerifyResult};
use crate::{Cli, Commands};
use anyhow::{Context, Result};
use std::path::PathBuf;

pub fn run(cli: Cli) -> Result<()> {
    let project_root = std::env::current_dir()
        .context("Failed to get current directory")?;

    // Parse info.toml
    let info_path = project_root.join("info.toml");
    let info = InfoFile::parse(&info_path)?;

    // Load state
    let mut state = AppState::load(&project_root, info.exercises)?;

    match cli.command {
        None => cmd_watch(&project_root, &mut state),
        Some(Commands::List) => cmd_list(&state),
        Some(Commands::Hint) => cmd_hint(&project_root, &state),
        Some(Commands::Verify { name }) => {
            cmd_verify(&project_root, &mut state, name.as_deref())
        }
        Some(Commands::Reset { name }) => cmd_reset(&project_root, &state, &name),
        Some(Commands::Explain { name }) => {
            cmd_explain(&project_root, &state, name.as_deref())
        }
        Some(Commands::Graph) => cmd_graph(&state),
    }
}

fn cmd_watch(_project_root: &PathBuf, _state: &mut AppState) -> Result<()> {
    output::print_info("Watch mode is not yet implemented. Use 'ros2lings verify' for now.");
    Ok(())
}

fn cmd_list(state: &AppState) -> Result<()> {
    let mut current_module = String::new();

    for (i, ex) in state.exercises.iter().enumerate() {
        if ex.module != current_module {
            current_module = ex.module.clone();
            println!("\n  {} {}", "──", current_module);
        }

        let status = if state.is_done(&ex.name) {
            "\x1b[32m✓\x1b[0m"
        } else if i == state.current_index {
            "\x1b[33m❯\x1b[0m"
        } else {
            "\x1b[90m✗\x1b[0m"
        };

        let lang = match ex.language {
            crate::info_file::Language::Cpp => "C++",
            crate::info_file::Language::Python => "Py ",
            crate::info_file::Language::C => "C  ",
        };

        println!(
            "  {} {:30} [{:3}] {:10} ~{}min",
            status,
            ex.name,
            lang,
            format!("{:?}", ex.mode).to_lowercase(),
            ex.estimated_minutes,
        );
    }

    let (done, total) = state.progress();
    println!();
    output::print_progress_bar(done, total);
    println!();

    Ok(())
}

fn cmd_hint(project_root: &PathBuf, state: &AppState) -> Result<()> {
    let exercise = state.current_exercise();
    let exercises_root = project_root.join("exercises");

    // Simple progressive hint: start with 0 (quick hint), then 1, 2, 3
    // For MVP, always show hint level 0 first, user runs `hint` again for deeper hints
    // TODO: Track hint level per exercise in state
    let (level, content) =
        crate::hint::show_hint(exercise, &exercises_root, 0)?;

    output::print_exercise_header(
        &exercise.name,
        &exercise.module,
        &format!("{:?}", exercise.mode).to_lowercase(),
        exercise.difficulty,
    );
    println!("{}", content);

    if level == 0 && exercise.hint_count > 0 {
        output::print_info(&format!(
            "Run 'ros2lings hint' again for more detailed hints ({} available)",
            exercise.hint_count
        ));
    }

    Ok(())
}

fn cmd_verify(
    project_root: &PathBuf,
    state: &mut AppState,
    name: Option<&str>,
) -> Result<()> {
    let ros2_env = Ros2Env::detect()?;

    let exercise_index = match name {
        Some(n) => state
            .find_exercise(n)
            .with_context(|| format!("Exercise '{}' not found", n))?,
        None => state.current_index,
    };

    let info = state.exercises[exercise_index].clone();
    let exercises_root = project_root.join("exercises");
    let exercise = Exercise::new(info.clone(), exercises_root);

    output::print_exercise_header(
        &info.name,
        &info.module,
        &format!("{:?}", info.mode).to_lowercase(),
        info.difficulty,
    );

    let pipeline = VerifyPipeline::new(ros2_env, project_root.clone());
    let result = pipeline.verify(&exercise)?;

    match result {
        VerifyResult::NotReady => {
            output::print_warning(
                "Exercise still has '// I AM NOT DONE' marker. \
                 Remove it when you're ready to verify.",
            );
        }
        VerifyResult::BuildFailed(output_text) => {
            output::print_error("Build failed!");
            println!();
            let formatted = verify::format_build_error(&output_text);
            println!("{}", formatted);
            println!();
            output::print_info("Try 'ros2lings hint' for help.");
        }
        VerifyResult::TestFailed(output_text) => {
            output::print_error("Tests failed!");
            println!();
            println!("{}", output_text);
            println!();
            output::print_info("Try 'ros2lings hint' for help.");
        }
        VerifyResult::Success => {
            output::print_success(&format!("Exercise '{}' passed!", info.name));
            if exercise_index == state.current_index {
                state.done_current_exercise()?;
                if state.all_done() {
                    println!();
                    output::print_success(
                        "Congratulations! You've completed all exercises!",
                    );
                } else {
                    let next = state.current_exercise();
                    output::print_info(&format!(
                        "Next up: {} ({})",
                        next.name, next.module
                    ));
                }
            }
        }
    }

    Ok(())
}

fn cmd_reset(project_root: &PathBuf, state: &AppState, name: &str) -> Result<()> {
    let idx = state
        .find_exercise(name)
        .with_context(|| format!("Exercise '{}' not found", name))?;
    let info = &state.exercises[idx];

    let exercises_dir = project_root.join("exercises").join(&info.dir);
    let solutions_dir = project_root.join("solutions").join(&info.dir);

    if !solutions_dir.exists() {
        anyhow::bail!(
            "Solution not found for '{}' at {}",
            name,
            solutions_dir.display()
        );
    }

    // Copy solution src/ files back to exercises
    let sol_src = solutions_dir.join("src");
    let ex_src = exercises_dir.join("src");

    if sol_src.is_dir() {
        for entry in std::fs::read_dir(&sol_src)? {
            let entry = entry?;
            let dest = ex_src.join(entry.file_name());
            std::fs::copy(entry.path(), &dest)
                .with_context(|| format!("Failed to reset {}", dest.display()))?;
        }
    }

    output::print_success(&format!("Exercise '{}' has been reset.", name));
    Ok(())
}

fn cmd_explain(
    project_root: &PathBuf,
    state: &AppState,
    name: Option<&str>,
) -> Result<()> {
    let exercise = match name {
        Some(n) => {
            let idx = state
                .find_exercise(n)
                .with_context(|| format!("Exercise '{}' not found", n))?;
            &state.exercises[idx]
        }
        None => state.current_exercise(),
    };

    let exercises_root = project_root.join("exercises");
    let content = crate::explain::get_explanation(exercise, &exercises_root)?;

    // Use termimad to render markdown in terminal
    let skin = termimad::MadSkin::default();
    skin.print_text(&content);

    Ok(())
}

fn cmd_graph(state: &AppState) -> Result<()> {
    let (done, total) = state.progress();

    println!();
    println!("  ROS2lings Learning Progress");
    println!("  ═══════════════════════════");
    println!();
    output::print_progress_bar(done, total);
    println!();

    // Group by module
    let mut modules: Vec<(String, usize, usize)> = Vec::new();
    for ex in &state.exercises {
        if let Some(m) = modules.last_mut() {
            if m.0 == ex.module {
                m.1 += 1;
                if state.is_done(&ex.name) {
                    m.2 += 1;
                }
                continue;
            }
        }
        let is_done = if state.is_done(&ex.name) { 1 } else { 0 };
        modules.push((ex.module.clone(), 1, is_done));
    }

    for (module, total_count, done_count) in &modules {
        let pct = if *total_count > 0 {
            (*done_count as f64 / *total_count as f64) * 100.0
        } else {
            0.0
        };
        let bar_len = 20;
        let filled = (*done_count * bar_len) / total_count.max(&1);
        let empty = bar_len - filled;
        println!(
            "  {:25} [{}{}] {}/{} ({:.0}%)",
            module,
            "█".repeat(filled),
            "░".repeat(empty),
            done_count,
            total_count,
            pct,
        );
    }
    println!();

    Ok(())
}
```

**Step 4: Add new modules to main.rs**

Add to top of `src/main.rs`:

```rust
mod hint;
mod explain;
```

**Step 5: Run all tests**

```bash
cargo test
```
Expected: All tests PASS

**Step 6: Verify it compiles**

```bash
cargo build
```

**Step 7: Commit**

```bash
git add src/cmd.rs src/hint.rs src/explain.rs src/output.rs src/main.rs
git commit -m "feat: implement all CLI commands (list, hint, verify, reset, explain, graph)"
```

---

## Phase 5: Watch Mode

### Task 10: File watcher with notify + event loop

**Files:**
- Create: `src/watch.rs`
- Modify: `src/cmd.rs` (wire up watch)
- Modify: `src/main.rs` (add `mod watch`)

**Step 1: Implement watch module**

Create `src/watch.rs`:

```rust
use crate::app_state::AppState;
use crate::exercise::Exercise;
use crate::output;
use crate::ros2_env::Ros2Env;
use crate::verify::{VerifyPipeline, VerifyResult};
use anyhow::Result;
use crossterm::event::{self, Event, KeyCode, KeyEvent};
use notify_debouncer_mini::{new_debouncer, DebouncedEventKind};
use std::path::PathBuf;
use std::sync::mpsc;
use std::time::Duration;

enum WatchEvent {
    FileChanged(PathBuf),
    Key(KeyEvent),
}

pub fn run_watch(project_root: &PathBuf, state: &mut AppState) -> Result<()> {
    let ros2_env = Ros2Env::detect()?;
    let pipeline = VerifyPipeline::new(ros2_env, project_root.clone());
    let exercises_root = project_root.join("exercises");

    // Show welcome
    println!();
    output::print_info("ROS2lings — Watch Mode");
    output::print_info("Watching for file changes... (press 'q' to quit, 'h' for hint, 'l' to list)");
    println!();

    show_current_exercise(state);

    // Set up file watcher
    let (tx, rx) = mpsc::channel();
    let tx_file = tx.clone();

    let mut debouncer = new_debouncer(
        Duration::from_millis(300),
        move |events: Result<Vec<notify_debouncer_mini::DebouncedEvent>, _>| {
            if let Ok(events) = events {
                for event in events {
                    if event.kind == DebouncedEventKind::Any {
                        let _ = tx_file.send(WatchEvent::FileChanged(event.path));
                    }
                }
            }
        },
    )?;

    debouncer.watcher().watch(
        &exercises_root,
        notify::RecursiveMode::Recursive,
    )?;

    // Keyboard input thread
    let tx_key = tx.clone();
    std::thread::spawn(move || loop {
        if let Ok(Event::Key(key)) = event::read() {
            let _ = tx_key.send(WatchEvent::Key(key));
        }
    });

    // Main event loop
    loop {
        match rx.recv() {
            Ok(WatchEvent::FileChanged(path)) => {
                // Check if the changed file belongs to the current exercise
                let current = state.current_exercise();
                let current_dir = exercises_root.join(&current.dir);

                if path.starts_with(&current_dir) {
                    // Check if it's a source file
                    if let Some(ext) = path.extension() {
                        match ext.to_str() {
                            Some("cpp" | "py" | "c" | "h" | "hpp") => {
                                println!();
                                output::print_info(&format!(
                                    "File changed: {}",
                                    path.file_name()
                                        .unwrap_or_default()
                                        .to_string_lossy()
                                ));
                                run_verify(
                                    &pipeline,
                                    state,
                                    &exercises_root,
                                )?;
                            }
                            _ => {}
                        }
                    }
                }
            }
            Ok(WatchEvent::Key(key)) => match key.code {
                KeyCode::Char('q') => {
                    println!();
                    output::print_info("Goodbye! Keep learning ROS2!");
                    break;
                }
                KeyCode::Char('h') => {
                    let exercise = state.current_exercise();
                    match crate::hint::show_hint(exercise, &exercises_root, 0) {
                        Ok((_, content)) => {
                            println!();
                            println!("{}", content);
                        }
                        Err(e) => output::print_error(&format!("Hint error: {}", e)),
                    }
                }
                KeyCode::Char('l') => {
                    show_current_exercise(state);
                }
                KeyCode::Char('n') => {
                    output::print_info("Skipping to next exercise...");
                    state.done_current_exercise()?;
                    show_current_exercise(state);
                }
                _ => {}
            },
            Err(_) => break,
        }
    }

    Ok(())
}

fn show_current_exercise(state: &AppState) {
    let ex = state.current_exercise();
    output::print_exercise_header(
        &ex.name,
        &ex.module,
        &format!("{:?}", ex.mode).to_lowercase(),
        ex.difficulty,
    );
    let (done, total) = state.progress();
    output::print_progress_bar(done, total);
    println!();
    let exercises_root = std::env::current_dir()
        .unwrap_or_default()
        .join("exercises");
    let source_dir = exercises_root.join(&ex.dir).join("src");
    output::print_info(&format!(
        "Open the exercise: {}",
        source_dir.display()
    ));
    println!();
}

fn run_verify(
    pipeline: &VerifyPipeline,
    state: &mut AppState,
    exercises_root: &PathBuf,
) -> Result<()> {
    let info = state.current_exercise().clone();
    let exercise = Exercise::new(info.clone(), exercises_root.clone());

    output::print_info("Compiling and testing...");

    match pipeline.verify(&exercise)? {
        VerifyResult::NotReady => {
            output::print_warning(
                "Remove '// I AM NOT DONE' when you're ready to verify.",
            );
        }
        VerifyResult::BuildFailed(out) => {
            output::print_error("Build failed!");
            let formatted = crate::verify::format_build_error(&out);
            println!("{}", formatted);
        }
        VerifyResult::TestFailed(out) => {
            output::print_error("Tests failed!");
            println!("{}", out);
        }
        VerifyResult::Success => {
            output::print_success(&format!("Exercise '{}' passed!", info.name));
            state.done_current_exercise()?;
            if state.all_done() {
                output::print_success(
                    "🎉 Congratulations! You've completed all exercises!",
                );
            } else {
                show_current_exercise(state);
            }
        }
    }

    Ok(())
}
```

**Step 2: Wire up in cmd.rs**

Replace the `cmd_watch` function in `src/cmd.rs`:

```rust
fn cmd_watch(project_root: &PathBuf, state: &mut AppState) -> Result<()> {
    crate::watch::run_watch(project_root, state)
}
```

**Step 3: Add module to main.rs**

Add `mod watch;` to `src/main.rs`.

**Step 4: Verify it compiles**

```bash
cargo build
```
Expected: BUILD SUCCESS

**Step 5: Commit**

```bash
git add src/watch.rs src/cmd.rs src/main.rs
git commit -m "feat: implement watch mode with file monitoring and keyboard controls"
```

---

## Phase 6: First Exercise (End-to-End Validation)

### Task 11: Create info.toml with Module 01-02 metadata

**Files:**
- Create: `info.toml`

**Step 1: Write the complete info.toml**

Create `info.toml` at project root with all 27 exercises defined. See the exercise tables in the design doc for the full list. Each exercise entry follows this pattern:

```toml
format_version = 1

welcome_message = """
🤖 欢迎来到 ROS2lings！

你将通过修复一系列"坏掉的" ROS2 程序来学习 ROS2 核心概念。
每个练习都是一个独立的 ROS2 包，包含 TODO 标记指引你修改。

快捷键: q=退出, h=提示, l=当前练习, n=跳过
"""

final_message = """
🎉 恭喜！你已经完成了 ROS2lings 的所有练习！

你现在掌握了:
- ROS2 节点、话题、发布/订阅
- 服务、Action 客户端/服务器
- 自定义消息、QoS、组件化节点

继续探索 ROS2 的更深层次吧！
"""

# ── Module 01: Nodes & Topics (15 exercises) ──

[[exercises]]
name = "01_hello_node"
dir = "01_nodes/01_hello_node"
module = "Nodes & Topics"
mode = "fix"
language = "cpp"
difficulty = 1
estimated_minutes = 5
hint_count = 3
test = true
hint = "rclcpp::init() 需要 argc 和 argv 两个参数。节点要用 std::make_shared 创建。"

[[exercises]]
name = "02_first_publisher"
dir = "01_nodes/02_first_publisher"
module = "Nodes & Topics"
mode = "fix"
language = "cpp"
difficulty = 1
estimated_minutes = 10
hint_count = 3
depends_on = ["01_hello_node"]
test = true
hint = "create_publisher 需要话题名和 QoS 深度。Timer 回调用 lambda 或 std::bind。"

[[exercises]]
name = "03_first_subscriber"
dir = "01_nodes/03_first_subscriber"
module = "Nodes & Topics"
mode = "fix"
language = "cpp"
difficulty = 1
estimated_minutes = 10
hint_count = 3
depends_on = ["02_first_publisher"]
test = true
hint = "create_subscription 需要话题名、QoS 和回调函数。回调参数类型是消息的 SharedPtr。"

[[exercises]]
name = "04_pubsub_connect"
dir = "01_nodes/04_pubsub_connect"
module = "Nodes & Topics"
mode = "implement"
language = "cpp"
difficulty = 2
estimated_minutes = 15
hint_count = 3
depends_on = ["03_first_subscriber"]
test = true
hint = "你需要创建一个发布者节点和一个订阅者节点，让它们通过同一个话题通信。"

[[exercises]]
name = "05_custom_message"
dir = "01_nodes/05_custom_message"
module = "Nodes & Topics"
mode = "implement"
language = "cpp"
difficulty = 2
estimated_minutes = 15
hint_count = 3
depends_on = ["04_pubsub_connect"]
test = true
hint = "在 msg/ 目录下创建 .msg 文件，在 CMakeLists.txt 中用 rosidl_generate_interfaces 生成。"

[[exercises]]
name = "06_multi_topic"
dir = "01_nodes/06_multi_topic"
module = "Nodes & Topics"
mode = "implement"
language = "cpp"
difficulty = 2
estimated_minutes = 20
hint_count = 3
depends_on = ["05_custom_message"]
test = true
hint = "一个节点可以同时有多个 publisher 和 subscriber，分别订阅不同的话题。"

[[exercises]]
name = "07_topic_introspection"
dir = "01_nodes/07_topic_introspection"
module = "Nodes & Topics"
mode = "explore"
language = "cpp"
difficulty = 1
estimated_minutes = 10
hint_count = 2
depends_on = ["04_pubsub_connect"]
test = true
hint = "使用 ros2 topic list、ros2 topic info、ros2 topic echo 来检查话题。"

[[exercises]]
name = "08_node_lifecycle_basics"
dir = "01_nodes/08_node_lifecycle_basics"
module = "Nodes & Topics"
mode = "explore"
language = "cpp"
difficulty = 2
estimated_minutes = 15
hint_count = 2
depends_on = ["01_hello_node"]
test = true
hint = "rclcpp::Node 构造函数内部会创建 rcl_node_t，注册到 context，分配回调组。"

[[exercises]]
name = "09_hello_node_py"
dir = "01_nodes/09_hello_node_py"
module = "Nodes & Topics"
mode = "fix"
language = "python"
difficulty = 1
estimated_minutes = 5
hint_count = 3
test = true
hint = "rclpy.init() 初始化，Node('name') 创建节点，rclpy.spin(node) 运行。"

[[exercises]]
name = "10_publisher_py"
dir = "01_nodes/10_publisher_py"
module = "Nodes & Topics"
mode = "fix"
language = "python"
difficulty = 1
estimated_minutes = 10
hint_count = 3
depends_on = ["09_hello_node_py"]
test = true
hint = "self.create_publisher(String, 'topic', 10) 创建发布者。Timer 用 self.create_timer。"

[[exercises]]
name = "11_subscriber_py"
dir = "01_nodes/11_subscriber_py"
module = "Nodes & Topics"
mode = "fix"
language = "python"
difficulty = 1
estimated_minutes = 10
hint_count = 3
depends_on = ["10_publisher_py"]
test = true
hint = "self.create_subscription(String, 'topic', callback, 10) 创建订阅者。"

[[exercises]]
name = "12_qos_mismatch"
dir = "01_nodes/12_qos_mismatch"
module = "Nodes & Topics"
mode = "debug"
language = "cpp"
difficulty = 2
estimated_minutes = 15
hint_count = 3
depends_on = ["03_first_subscriber"]
test = true
hint = "Publisher 和 Subscriber 的 QoS 策略必须兼容。检查 reliability 和 durability 设置。"

[[exercises]]
name = "13_component_node"
dir = "01_nodes/13_component_node"
module = "Nodes & Topics"
mode = "implement"
language = "cpp"
difficulty = 3
estimated_minutes = 20
hint_count = 3
depends_on = ["04_pubsub_connect"]
test = true
hint = "组件节点继承 rclcpp::Node，在构造函数中初始化，用 RCLCPP_COMPONENTS_REGISTER_NODE 注册。"

[[exercises]]
name = "14_namespace_remap"
dir = "01_nodes/14_namespace_remap"
module = "Nodes & Topics"
mode = "fix"
language = "cpp"
difficulty = 2
estimated_minutes = 10
hint_count = 3
depends_on = ["04_pubsub_connect"]
test = true
hint = "NodeOptions 可以设置命名空间。话题重映射通过 remap 参数实现。"

[[exercises]]
name = "15_multi_node_process"
dir = "01_nodes/15_multi_node_process"
module = "Nodes & Topics"
mode = "implement"
language = "cpp"
difficulty = 2
estimated_minutes = 15
hint_count = 3
depends_on = ["04_pubsub_connect"]
test = true
hint = "多个节点可以共享一个 Executor。用 executor.add_node() 添加，executor.spin() 运行。"

# ── Module 02: Services & Actions (12 exercises) ──

[[exercises]]
name = "16_first_service"
dir = "02_services/01_first_service"
module = "Services & Actions"
mode = "fix"
language = "cpp"
difficulty = 1
estimated_minutes = 10
hint_count = 3
test = true
hint = "create_service 需要服务名和回调。回调签名: (request, response) -> void。"

[[exercises]]
name = "17_service_client"
dir = "02_services/02_service_client"
module = "Services & Actions"
mode = "fix"
language = "cpp"
difficulty = 2
estimated_minutes = 15
hint_count = 3
depends_on = ["16_first_service"]
test = true
hint = "异步调用: client->async_send_request(request)。用 rclcpp::spin_until_future_complete 等待。"

[[exercises]]
name = "18_service_pair"
dir = "02_services/03_service_pair"
module = "Services & Actions"
mode = "implement"
language = "cpp"
difficulty = 2
estimated_minutes = 20
hint_count = 3
depends_on = ["17_service_client"]
test = true
hint = "实现一个 AddTwoInts 服务: server 接收两个整数并返回它们的和。"

[[exercises]]
name = "19_custom_srv"
dir = "02_services/04_custom_srv"
module = "Services & Actions"
mode = "implement"
language = "cpp"
difficulty = 2
estimated_minutes = 15
hint_count = 3
depends_on = ["18_service_pair"]
test = true
hint = "在 srv/ 目录下创建 .srv 文件。格式: request 字段 --- response 字段。"

[[exercises]]
name = "20_service_py"
dir = "02_services/05_service_py"
module = "Services & Actions"
mode = "fix"
language = "python"
difficulty = 1
estimated_minutes = 10
hint_count = 3
test = true
hint = "self.create_service(AddTwoInts, 'service_name', callback)。回调: (request, response) -> response。"

[[exercises]]
name = "21_first_action_server"
dir = "02_services/06_first_action_server"
module = "Services & Actions"
mode = "fix"
language = "cpp"
difficulty = 3
estimated_minutes = 20
hint_count = 3
depends_on = ["16_first_service"]
test = true
hint = "ActionServer 需要 handle_goal、handle_cancel、handle_accepted 三个回调。"

[[exercises]]
name = "22_action_client"
dir = "02_services/07_action_client"
module = "Services & Actions"
mode = "fix"
language = "cpp"
difficulty = 3
estimated_minutes = 20
hint_count = 3
depends_on = ["21_first_action_server"]
test = true
hint = "async_send_goal 发送目标。通过 GoalHandleFibonacci::SharedPtr 获取反馈和结果。"

[[exercises]]
name = "23_action_complete"
dir = "02_services/08_action_complete"
module = "Services & Actions"
mode = "implement"
language = "cpp"
difficulty = 3
estimated_minutes = 25
hint_count = 3
depends_on = ["22_action_client"]
test = true
hint = "实现完整的 Fibonacci Action: server 计算并发送中间反馈，client 可以取消目标。"

[[exercises]]
name = "24_custom_action"
dir = "02_services/09_custom_action"
module = "Services & Actions"
mode = "implement"
language = "cpp"
difficulty = 2
estimated_minutes = 15
hint_count = 3
depends_on = ["23_action_complete"]
test = true
hint = "在 action/ 目录下创建 .action 文件。格式: goal --- result --- feedback。"

[[exercises]]
name = "25_action_py"
dir = "02_services/10_action_py"
module = "Services & Actions"
mode = "fix"
language = "python"
difficulty = 2
estimated_minutes = 15
hint_count = 3
test = true
hint = "ActionServer(self, Fibonacci, 'fibonacci', execute_callback=self.execute_callback)。"

[[exercises]]
name = "26_service_introspection"
dir = "02_services/11_service_introspection"
module = "Services & Actions"
mode = "explore"
language = "cpp"
difficulty = 2
estimated_minutes = 10
hint_count = 2
depends_on = ["18_service_pair"]
test = true
hint = "使用 ros2 service list、ros2 service type、ros2 service call 检查服务。"

[[exercises]]
name = "27_action_state_machine"
dir = "02_services/12_action_state_machine"
module = "Services & Actions"
mode = "debug"
language = "cpp"
difficulty = 3
estimated_minutes = 20
hint_count = 3
depends_on = ["23_action_complete"]
test = true
hint = "Action 有严格的状态转换: ACCEPTED → EXECUTING → SUCCEEDED/ABORTED/CANCELED。"
```

**Step 2: Verify parsing works**

```bash
cargo run -- list
```
Expected: Shows all 27 exercises listed (will fail to load state but should parse info.toml)

**Step 3: Commit**

```bash
git add info.toml
git commit -m "feat: add info.toml with all 27 MVP exercises metadata"
```

---

### Task 12: Create first exercise — 01_hello_node (end-to-end validation)

This is the critical task that validates the entire pipeline works end-to-end.

**Files:**
- Create: `exercises/01_nodes/01_hello_node/src/hello_node.cpp`
- Create: `exercises/01_nodes/01_hello_node/test/test_hello_node.cpp`
- Create: `exercises/01_nodes/01_hello_node/hints/hint1.md`
- Create: `exercises/01_nodes/01_hello_node/hints/hint2.md`
- Create: `exercises/01_nodes/01_hello_node/hints/hint3.md`
- Create: `exercises/01_nodes/01_hello_node/explain.md`
- Create: `exercises/01_nodes/01_hello_node/CMakeLists.txt`
- Create: `exercises/01_nodes/01_hello_node/package.xml`
- Create: `solutions/01_nodes/01_hello_node/src/hello_node.cpp`

**Step 1: Create directory structure**

```bash
mkdir -p exercises/01_nodes/01_hello_node/{src,test,hints}
mkdir -p solutions/01_nodes/01_hello_node/src
```

**Step 2: Write the broken exercise source**

`exercises/01_nodes/01_hello_node/src/hello_node.cpp`:

```cpp
// I AM NOT DONE
//
// 练习: hello_node
// 模块: 01 - Nodes & Topics
// 难度: ★☆☆☆☆
//
// 学习目标:
//   理解 ROS2 节点的创建和 rclcpp::init/shutdown 生命周期。
//
// 说明:
//   下面的代码尝试创建一个最简单的 ROS2 节点并让它打印一条消息。
//   但代码中有几个错误需要你修复。
//
// 步骤:
//   1. 修复 rclcpp::init() 的参数
//   2. 修复节点的创建方式（提示：需要智能指针）
//   3. 确保节点能正确 spin 一次
//   4. 修复完成后，删除文件顶部的 "// I AM NOT DONE"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  // TODO: rclcpp::init() 需要 argc 和 argv 来初始化 ROS2 上下文
  rclcpp::init();

  // TODO: 节点需要用 std::make_shared 创建，而不是直接构造
  auto node = rclcpp::Node("hello_node");

  RCLCPP_INFO(node->get_logger(), "Hello, ROS2! I am a node.");

  // TODO: 让节点至少运行一次（提示：spin_some 或 spin_once）
  rclcpp::shutdown();
  return 0;
}
```

**Step 3: Write the test**

`exercises/01_nodes/01_hello_node/test/test_hello_node.cpp`:

```cpp
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <memory>

// 这个测试文件是只读的——你不需要修改它。
// 它会自动验证你的修复是否正确。

class HelloNodeTest : public ::testing::Test {
protected:
  void SetUp() override {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }
  void TearDown() override {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

TEST_F(HelloNodeTest, CanCreateNode) {
  // 验证: 能正确创建一个节点
  auto node = std::make_shared<rclcpp::Node>("test_hello_node");
  ASSERT_NE(node, nullptr);
}

TEST_F(HelloNodeTest, NodeHasCorrectName) {
  // 验证: 节点名称正确
  auto node = std::make_shared<rclcpp::Node>("test_hello_node");
  EXPECT_EQ(std::string(node->get_name()), "test_hello_node");
}

TEST_F(HelloNodeTest, CanSpinOnce) {
  // 验证: 节点能正确 spin
  auto node = std::make_shared<rclcpp::Node>("test_spin_node");
  rclcpp::spin_some(node);
  // If we get here without crashing, spin works
  SUCCEED();
}
```

**Step 4: Write CMakeLists.txt**

`exercises/01_nodes/01_hello_node/CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.8)
project(ros2lings_01_hello_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# 练习可执行文件
add_executable(hello_node src/hello_node.cpp)
ament_target_dependencies(hello_node rclcpp)
install(TARGETS hello_node DESTINATION lib/${PROJECT_NAME})

# 测试
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_hello_node test/test_hello_node.cpp)
  ament_target_dependencies(test_hello_node rclcpp)
endif()

ament_package()
```

**Step 5: Write package.xml**

`exercises/01_nodes/01_hello_node/package.xml`:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>ros2lings_01_hello_node</name>
  <version>0.1.0</version>
  <description>ROS2lings Exercise: hello_node — 理解节点创建和生命周期</description>
  <maintainer email="ros2lings@example.com">ros2lings</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>rclcpp</depend>

  <test_depend>ament_cmake_gtest</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

**Step 6: Write hints**

`exercises/01_nodes/01_hello_node/hints/hint1.md`:

```markdown
# 提示 1 — 方向性

这个练习有三个错误，都跟 ROS2 节点的生命周期有关：

1. **初始化**: `rclcpp::init()` 需要知道命令行参数
2. **创建**: ROS2 节点通常用智能指针管理
3. **运行**: 节点需要"转一下"才能处理事件

想想 `main` 函数已经给你了什么参数？
```

`exercises/01_nodes/01_hello_node/hints/hint2.md`:

```markdown
# 提示 2 — 具体线索

1. `rclcpp::init(argc, argv)` — 传入命令行参数
2. `std::make_shared<rclcpp::Node>("name")` — 用智能指针创建节点
3. `rclcpp::spin_some(node)` — 让节点处理一轮回调

节点的创建和 spin 都需要 `shared_ptr`，这是 ROS2 的约定。
```

`exercises/01_nodes/01_hello_node/hints/hint3.md`:

```markdown
# 提示 3 — 几乎是答案

把三行代码改成：

```cpp
rclcpp::init(argc, argv);
auto node = std::make_shared<rclcpp::Node>("hello_node");
// ... RCLCPP_INFO ...
rclcpp::spin_some(node);
rclcpp::shutdown();
```

`rclcpp::spin_some()` 会处理当前等待的所有回调然后返回，
而 `rclcpp::spin()` 会一直阻塞直到节点被关闭。
```

**Step 7: Write explain.md**

`exercises/01_nodes/01_hello_node/explain.md`:

```markdown
# 节点（Node）— ROS2 的基本计算单元

## 概念

Node 是 ROS2 中最基础的运行单元。每个节点是一个独立的功能模块，
通过话题（Topic）、服务（Service）、Action 与其他节点通信。

一个机器人系统通常由数十个节点组成：相机驱动节点、目标检测节点、
路径规划节点、运动控制节点……它们各司其职，通过 ROS2 中间件通信。

## 生命周期

```
rclcpp::init(argc, argv)
        │
        ▼
std::make_shared<rclcpp::Node>("name")
        │
        ▼
  注册发布者/订阅者/服务/定时器
        │
        ▼
   rclcpp::spin(node)  ← 事件循环，处理回调
        │
        ▼
  rclcpp::shutdown()   ← 清理资源
```

## 架构深度

你写的 `rclcpp::Node` 背后发生了什么：

```
rclcpp::Node (C++ 用户层)
    │
    ├── 创建 rcl_node_t (C 接口层)
    │     └── 验证节点名合法性
    │     └── 分配默认回调组
    │
    ├── 注册到 rclcpp::Context
    │     └── 全局上下文管理所有节点
    │
    └── 通过 rmw 层创建 DDS Participant
          └── 开始 DDS 自动发现
```

## 关键点

- `rclcpp::init()` 初始化全局 Context，一个进程只调用一次
- 节点用 `shared_ptr` 管理是因为 Executor 和回调都需要持有引用
- `spin()` 阻塞式事件循环；`spin_some()` 处理当前队列后返回
- `shutdown()` 清理所有节点和 DDS 资源
```

**Step 8: Write the solution**

`solutions/01_nodes/01_hello_node/src/hello_node.cpp`:

```cpp
// 参考答案 — hello_node
//
// 这是修复后的完整代码。
// 学习者的文件初始状态有三个错误：
// 1. rclcpp::init() 缺少参数
// 2. 节点创建方式错误（未用 make_shared）
// 3. 缺少 spin_some 调用

#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("hello_node");

  RCLCPP_INFO(node->get_logger(), "Hello, ROS2! I am a node.");

  rclcpp::spin_some(node);
  rclcpp::shutdown();
  return 0;
}
```

**Step 9: Test end-to-end locally**

```bash
# Source ROS2
source /opt/ros/humble/setup.bash

# Build the exercise (should fail because of bugs)
colcon build --paths exercises/01_nodes/01_hello_node \
  --packages-select ros2lings_01_hello_node \
  --build-base /tmp/ros2lings_build \
  --install-base /tmp/ros2lings_install

# Copy solution and rebuild (should succeed)
cp solutions/01_nodes/01_hello_node/src/hello_node.cpp \
   exercises/01_nodes/01_hello_node/src/hello_node.cpp

colcon build --paths exercises/01_nodes/01_hello_node \
  --packages-select ros2lings_01_hello_node \
  --build-base /tmp/ros2lings_build \
  --install-base /tmp/ros2lings_install

# Run tests
colcon test --packages-select ros2lings_01_hello_node \
  --build-base /tmp/ros2lings_build \
  --install-base /tmp/ros2lings_install

# Check results
colcon test-result --build-base /tmp/ros2lings_build
```

Expected: Build fails with broken code, succeeds with solution, tests pass.

**Step 10: Restore broken exercise**

```bash
git checkout exercises/01_nodes/01_hello_node/src/hello_node.cpp
```

**Step 11: Commit**

```bash
git add exercises/01_nodes/01_hello_node/ solutions/01_nodes/01_hello_node/
git commit -m "feat: add exercise 01_hello_node — first ROS2 node (end-to-end)"
```

---

## Phase 7: Remaining Module 01 Exercises

### Task 13: Create exercises 02-08 (Module 01, C++ core)

Follow the same file structure pattern as Task 12 for each exercise. Each exercise needs:
- `exercises/<dir>/src/<name>.cpp` — broken source with `// I AM NOT DONE` and `// TODO`
- `exercises/<dir>/test/test_<name>.cpp` — gtest tests
- `exercises/<dir>/hints/hint1.md, hint2.md, hint3.md`
- `exercises/<dir>/explain.md`
- `exercises/<dir>/CMakeLists.txt`
- `exercises/<dir>/package.xml`
- `solutions/<dir>/src/<name>.cpp`

Create all 7 exercises in parallel, then test each one.

**Exercise 02: first_publisher** — Fix Timer + Publisher creation. Test verifies messages published.

**Exercise 03: first_subscriber** — Fix Subscription callback. Test verifies callback receives messages.

**Exercise 04: pubsub_connect** — Implement both publisher and subscriber. Test verifies end-to-end message flow.

**Exercise 05: custom_message** — Implement custom .msg + usage. Needs extra `msg/` dir and `rosidl_generate_interfaces` in CMakeLists.txt.

**Exercise 06: multi_topic** — Implement multi-topic node. Test verifies messages on multiple topics.

**Exercise 07: topic_introspection** — Explore exercise. Test verifies answer strings.

**Exercise 08: node_lifecycle_basics** — Explore exercise. Test verifies answer strings about Node internals.

**After creating all 7:**

```bash
# Verify each exercise's solution builds and passes tests
for ex in 02_first_publisher 03_first_subscriber 04_pubsub_connect 05_custom_message 06_multi_topic 07_topic_introspection 08_node_lifecycle_basics; do
  source /opt/ros/humble/setup.bash
  cp -r solutions/01_nodes/$ex/src/* exercises/01_nodes/$ex/src/ 2>/dev/null
  colcon build --paths exercises/01_nodes/$ex \
    --packages-select ros2lings_$ex \
    --build-base /tmp/ros2lings_build \
    --install-base /tmp/ros2lings_install
  colcon test --packages-select ros2lings_$ex \
    --build-base /tmp/ros2lings_build \
    --install-base /tmp/ros2lings_install
done
colcon test-result --build-base /tmp/ros2lings_build
```

**Commit after each exercise passes, or batch commit:**

```bash
git add exercises/01_nodes/ solutions/01_nodes/
git commit -m "feat: add Module 01 exercises 02-08 (C++ core topics)"
```

---

### Task 14: Create exercises 09-11 (Module 01, Python)

Python exercises use `ament_python` or `ament_cmake` with `ament_cmake_pytest`.

For Python exercises, the package structure differs:
- Use `ament_cmake` build type (consistent with C++ exercises)
- Python source in `src/<name>.py` (or `<name>/<name>.py` for ament_python)
- Tests use `pytest` via `ament_cmake_pytest`
- `setup.cfg` and `setup.py` if using ament_python

**Simpler approach for MVP:** Use `ament_cmake` with `install(PROGRAMS)` for Python scripts, and `ament_cmake_pytest` for tests. This keeps the build system consistent.

**Commit:**

```bash
git add exercises/01_nodes/09_hello_node_py exercises/01_nodes/10_publisher_py exercises/01_nodes/11_subscriber_py
git add solutions/01_nodes/09_hello_node_py solutions/01_nodes/10_publisher_py solutions/01_nodes/11_subscriber_py
git commit -m "feat: add Module 01 exercises 09-11 (Python nodes)"
```

---

### Task 15: Create exercises 12-15 (Module 01, advanced)

**Exercise 12: qos_mismatch** — Debug QoS incompatibility between pub/sub.

**Exercise 13: component_node** — Implement composable node. Needs `rclcpp_components` dependency.

**Exercise 14: namespace_remap** — Fix namespace and remap configuration.

**Exercise 15: multi_node_process** — Implement multiple nodes in one process with shared Executor.

**Commit:**

```bash
git add exercises/01_nodes/12_qos_mismatch exercises/01_nodes/13_component_node
git add exercises/01_nodes/14_namespace_remap exercises/01_nodes/15_multi_node_process
git add solutions/01_nodes/
git commit -m "feat: add Module 01 exercises 12-15 (QoS, components, namespaces, multi-node)"
```

---

## Phase 8: Module 02 Exercises

### Task 16: Create exercises 01-05 (Services)

**Exercise 16 (01_first_service):** Fix Service Server callback. Needs `example_interfaces` for `AddTwoInts`.

**Exercise 17 (02_service_client):** Fix async Client call.

**Exercise 18 (03_service_pair):** Implement complete Server + Client.

**Exercise 19 (04_custom_srv):** Implement custom .srv file. Needs `rosidl_generate_interfaces`.

**Exercise 20 (05_service_py):** Fix Python Service Server/Client.

**Commit:**

```bash
git add exercises/02_services/ solutions/02_services/
git commit -m "feat: add Module 02 exercises 01-05 (services)"
```

---

### Task 17: Create exercises 06-12 (Actions)

**Exercise 21 (06_first_action_server):** Fix Action Server. Needs `rclcpp_action` and `action_tutorials_interfaces` (or custom action).

**Exercise 22 (07_action_client):** Fix Action Client with feedback.

**Exercise 23 (08_action_complete):** Implement full Action with cancel support.

**Exercise 24 (09_custom_action):** Implement custom .action file.

**Exercise 25 (10_action_py):** Fix Python Action Server/Client.

**Exercise 26 (11_service_introspection):** Explore — answer questions about service introspection.

**Exercise 27 (12_action_state_machine):** Debug Action state machine issues.

**Commit:**

```bash
git add exercises/02_services/ solutions/02_services/
git commit -m "feat: add Module 02 exercises 06-12 (actions)"
```

---

## Phase 9: Integration & Polish

### Task 18: End-to-end integration test

**Step 1: Verify all exercises build and test**

```bash
source /opt/ros/humble/setup.bash

# Copy all solutions
for dir in solutions/01_nodes/*/src solutions/02_services/*/src; do
  target=$(echo $dir | sed 's|solutions/|exercises/|')
  cp -r $dir/* $target/ 2>/dev/null || true
done

# Build all
colcon build \
  --build-base /tmp/ros2lings_build \
  --install-base /tmp/ros2lings_install \
  --paths exercises/01_nodes/* exercises/02_services/*

# Test all
colcon test \
  --build-base /tmp/ros2lings_build \
  --install-base /tmp/ros2lings_install

# Check results
colcon test-result --build-base /tmp/ros2lings_build --verbose
```

Expected: ALL 27 exercises build and pass tests.

**Step 2: Restore all exercises to broken state**

```bash
git checkout exercises/
```

**Step 3: Test CLI commands**

```bash
cargo run -- list
cargo run -- hint
cargo run -- graph
cargo run -- explain 01_hello_node
```

**Step 4: Test watch mode manually**

```bash
# Terminal 1
cargo run

# Terminal 2 — edit an exercise, save, watch Terminal 1 for output
```

---

### Task 19: Run cargo tests

```bash
cargo test
```

Expected: All unit tests pass (info_file, exercise, app_state, hint, explain, verify, cli parsing).

---

### Task 20: Final commit and tag

```bash
git add -A
git commit -m "feat: ROS2lings MVP — CLI + Module 01-02 (27 exercises)

Complete MVP implementation:
- Rust CLI with watch mode, verify, list, hint, explain, reset, graph
- Module 01: Nodes & Topics (15 exercises, C++/Python)
- Module 02: Services & Actions (12 exercises, C++/Python)
- Progressive hints (3 levels), architecture explanations
- Colcon build/test integration with error formatting
- Progress persistence across sessions"

git tag v0.1.0-mvp
```
