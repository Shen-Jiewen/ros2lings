use crate::info_file::ExerciseInfo;
use anyhow::{Context, Result};
use std::path::Path;

pub fn show_hint(
    exercise: &ExerciseInfo,
    exercises_root: &Path,
    hint_level: u32,
) -> Result<(u32, String)> {
    if hint_level == 0 && !exercise.hint.is_empty() {
        return Ok((0, exercise.hint.clone()));
    }

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
            name: "test".to_string(), dir: "m/test".to_string(),
            module: "M".to_string(), mode: ExerciseMode::Fix,
            language: Language::Cpp, difficulty: 1, estimated_minutes: 5,
            hint_count: 1, depends_on: vec![], test: true,
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
            name: "test".to_string(), dir: "m/test".to_string(),
            module: "M".to_string(), mode: ExerciseMode::Fix,
            language: Language::Cpp, difficulty: 1, estimated_minutes: 5,
            hint_count: 2, depends_on: vec![], test: true,
            hint: "quick".to_string(),
        };
        let (level, content) = show_hint(&info, tmp.path(), 1).unwrap();
        assert_eq!(level, 1);
        assert!(content.contains("Detailed hint"));
    }
}
