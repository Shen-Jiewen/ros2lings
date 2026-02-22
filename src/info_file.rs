use anyhow::{Context, Result};
use serde::Deserialize;
use std::path::Path;

#[derive(Deserialize, Debug)]
#[allow(dead_code)]
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
    #[allow(dead_code)]
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
    Urdf,
    Xacro,
}

impl InfoFile {
    pub fn parse(path: &Path) -> Result<Self> {
        let content = std::fs::read_to_string(path)
            .with_context(|| format!("Failed to read info.toml at {}", path.display()))?;
        let info: InfoFile =
            toml::from_str(&content).with_context(|| "Failed to parse info.toml")?;

        if info.format_version != 1 {
            anyhow::bail!(
                "Unsupported info.toml format_version: {}. Expected 1.",
                info.format_version
            );
        }

        if info.exercises.is_empty() {
            anyhow::bail!("info.toml contains no exercises");
        }

        // Validate difficulty range (1–5) to prevent underflow in star rendering
        for ex in &info.exercises {
            if ex.difficulty == 0 || ex.difficulty > 5 {
                anyhow::bail!(
                    "Exercise '{}': difficulty {} out of range 1–5",
                    ex.name,
                    ex.difficulty
                );
            }
        }

        // Validate exercise name uniqueness
        let mut seen = std::collections::HashSet::new();
        for ex in &info.exercises {
            if !seen.insert(&ex.name) {
                anyhow::bail!("Duplicate exercise name '{}' in info.toml", ex.name);
            }
        }

        Ok(info)
    }
}

impl ExerciseInfo {
    pub fn package_name(&self) -> String {
        format!("ros2lings_{}", self.name)
    }

    #[allow(dead_code)]
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
        let toml_str = r#"
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
        let f = write_temp_toml(toml_str);
        let info = InfoFile::parse(f.path()).unwrap();
        assert_eq!(info.exercises.len(), 1);
        assert_eq!(info.exercises[0].name, "01_hello_node");
        assert_eq!(info.exercises[0].mode, ExerciseMode::Fix);
        assert_eq!(info.exercises[0].language, Language::Cpp);
        assert_eq!(info.exercises[0].difficulty, 1);
    }

    #[test]
    fn test_parse_multiple_exercises() {
        let toml_str = r#"
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
        let f = write_temp_toml(toml_str);
        let info = InfoFile::parse(f.path()).unwrap();
        assert_eq!(info.exercises.len(), 2);
        assert_eq!(
            info.exercises[1].depends_on,
            vec!["01_hello_node".to_string()]
        );
    }

    #[test]
    fn test_reject_wrong_format_version() {
        let toml_str = r#"
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
        let f = write_temp_toml(toml_str);
        let result = InfoFile::parse(f.path());
        assert!(result.is_err());
        assert!(result
            .unwrap_err()
            .to_string()
            .contains("Unsupported info.toml format_version"));
    }

    #[test]
    fn test_reject_empty_exercises() {
        let toml_str = r#"format_version = 1"#;
        let f = write_temp_toml(toml_str);
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
        let toml_str = r#"
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
        let f = write_temp_toml(toml_str);
        let info = InfoFile::parse(f.path()).unwrap();
        assert!(info.exercises[0].test);
    }

    #[test]
    fn test_all_modes_parse() {
        for mode in ["fix", "implement", "explore", "debug"] {
            let toml_str = format!(
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
            let f = write_temp_toml(&toml_str);
            assert!(InfoFile::parse(f.path()).is_ok(), "Failed for mode: {mode}");
        }
    }

    #[test]
    fn test_all_languages_parse() {
        for lang in ["cpp", "python", "c", "urdf", "xacro"] {
            let toml_str = format!(
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
            let f = write_temp_toml(&toml_str);
            assert!(
                InfoFile::parse(f.path()).is_ok(),
                "Failed for language: {lang}"
            );
        }
    }

    #[test]
    fn test_reject_difficulty_zero() {
        let toml_str = r#"
format_version = 1
[[exercises]]
name = "t"
dir = "t"
module = "t"
mode = "fix"
language = "cpp"
difficulty = 0
estimated_minutes = 1
hint_count = 1
hint = "h"
"#;
        let f = write_temp_toml(toml_str);
        let err = InfoFile::parse(f.path()).unwrap_err();
        assert!(err.to_string().contains("difficulty 0 out of range"));
    }

    #[test]
    fn test_reject_difficulty_too_high() {
        let toml_str = r#"
format_version = 1
[[exercises]]
name = "t"
dir = "t"
module = "t"
mode = "fix"
language = "cpp"
difficulty = 6
estimated_minutes = 1
hint_count = 1
hint = "h"
"#;
        let f = write_temp_toml(toml_str);
        let err = InfoFile::parse(f.path()).unwrap_err();
        assert!(err.to_string().contains("difficulty 6 out of range"));
    }

    #[test]
    fn test_reject_duplicate_exercise_names() {
        let toml_str = r#"
format_version = 1
[[exercises]]
name = "dup"
dir = "a"
module = "t"
mode = "fix"
language = "cpp"
difficulty = 1
estimated_minutes = 1
hint_count = 1
hint = "h"

[[exercises]]
name = "dup"
dir = "b"
module = "t"
mode = "fix"
language = "cpp"
difficulty = 1
estimated_minutes = 1
hint_count = 1
hint = "h"
"#;
        let f = write_temp_toml(toml_str);
        let err = InfoFile::parse(f.path()).unwrap_err();
        assert!(err.to_string().contains("Duplicate exercise name 'dup'"));
    }
}
