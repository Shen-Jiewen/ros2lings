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

    pub fn dir_path(&self) -> PathBuf {
        self.exercises_root.join(&self.info.dir)
    }

    pub fn source_file(&self) -> Result<PathBuf> {
        let dir = self.dir_path();
        let src_dir = dir.join("src");

        if src_dir.is_dir() {
            for entry in std::fs::read_dir(&src_dir)
                .with_context(|| format!("Failed to read {}", src_dir.display()))?
            {
                let entry = entry?;
                let path = entry.path();
                if let Some(ext) = path.extension() {
                    if let Some("cpp" | "py" | "c") = ext.to_str() {
                        return Ok(path);
                    }
                }
            }
        }

        for entry in
            std::fs::read_dir(&dir).with_context(|| format!("Failed to read {}", dir.display()))?
        {
            let entry = entry?;
            let path = entry.path();
            if let Some(ext) = path.extension() {
                if ext == "py" {
                    return Ok(path);
                }
            }
        }

        anyhow::bail!("No source file found in exercise '{}'", self.info.name)
    }

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
