use crate::info_file::ExerciseInfo;
use anyhow::{Context, Result};
use std::path::Path;

pub fn get_explanation(exercise: &ExerciseInfo, exercises_root: &Path) -> Result<String> {
    let explain_file = exercises_root.join(&exercise.dir).join("explain.md");

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
