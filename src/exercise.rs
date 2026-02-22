use crate::info_file::{ExerciseInfo, Language};
use anyhow::{Context, Result};
use std::path::{Path, PathBuf};

const DONE_MARKER: &str = "// I AM NOT DONE";
const DONE_MARKER_PY: &str = "# I AM NOT DONE";
const DONE_MARKER_XML: &str = "<!-- I AM NOT DONE -->";

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

    /// Returns the most relevant source file for the user to edit.
    /// Prefers the file that still has the done marker (the one needing work),
    /// falling back to the first candidate file.
    pub fn source_file(&self) -> Result<PathBuf> {
        let files = self.all_source_files()?;
        if files.is_empty() {
            anyhow::bail!("No source file found in exercise '{}'", self.info.name);
        }
        // Prefer the file with the done marker (the file the user needs to edit)
        for f in &files {
            if Self::has_done_marker(f).unwrap_or(false) {
                return Ok(f.clone());
            }
        }
        // No marker found — return the first candidate
        Ok(files.into_iter().next().unwrap())
    }

    /// Returns ALL candidate source files across all search directories for this language.
    pub fn all_source_files(&self) -> Result<Vec<PathBuf>> {
        let dir = self.dir_path();
        let search_plan = Self::search_plan_for(&self.info.language, &dir);

        let mut results = Vec::new();
        for (search_dir, extensions) in &search_plan {
            Self::collect_files_with_extensions(search_dir, extensions, &mut results)?;
        }
        Ok(results)
    }

    fn search_plan_for<'a>(language: &Language, dir: &Path) -> Vec<(PathBuf, &'a [&'a str])> {
        match language {
            Language::Cpp => vec![
                (dir.join("src"), &["cpp", "c"]),
            ],
            Language::C => vec![
                (dir.join("src"), &["c"]),
            ],
            Language::Python => vec![
                (dir.join("src"), &["py"]),
                (dir.join("launch"), &["py"]),
                (dir.to_path_buf(), &["py"]),
            ],
            Language::Urdf => vec![
                (dir.join("urdf"), &["urdf"]),
            ],
            Language::Xacro => vec![
                (dir.join("urdf"), &["xacro"]),
            ],
        }
    }

    fn collect_files_with_extensions(
        dir: &Path,
        extensions: &[&str],
        out: &mut Vec<PathBuf>,
    ) -> Result<()> {
        if !dir.is_dir() {
            return Ok(());
        }
        for entry in
            std::fs::read_dir(dir).with_context(|| format!("Failed to read {}", dir.display()))?
        {
            let entry = entry?;
            let path = entry.path();
            if let Some(ext) = path.extension() {
                if let Some(ext_str) = ext.to_str() {
                    if extensions.contains(&ext_str) {
                        out.push(path);
                    }
                }
            }
        }
        Ok(())
    }

    pub fn has_done_marker(source_path: &Path) -> Result<bool> {
        let content = std::fs::read_to_string(source_path)
            .with_context(|| format!("Failed to read {}", source_path.display()))?;
        // Match only standalone marker lines (trimmed), not substrings in instruction text.
        Ok(content.lines().any(|line| {
            let trimmed = line.trim();
            trimmed == DONE_MARKER || trimmed == DONE_MARKER_PY || trimmed == DONE_MARKER_XML
        }))
    }

    /// Checks ALL candidate source files for a done marker.
    /// Returns true if ANY file still contains the marker.
    pub fn any_done_marker(&self) -> Result<bool> {
        for path in self.all_source_files()? {
            if Self::has_done_marker(&path)? {
                return Ok(true);
            }
        }
        Ok(false)
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

    #[test]
    fn test_marker_in_instruction_text_not_false_positive() {
        let tmp = TempDir::new().unwrap();
        // Simulates a file where the top marker was removed but instruction text
        // still mentions the marker in quotes — must NOT be detected.
        let file = tmp.path().join("hello.cpp");
        fs::write(
            &file,
            "// steps:\n//   4. 删除顶部的 \"// I AM NOT DONE\"\nint main() {}",
        )
        .unwrap();
        assert!(!Exercise::has_done_marker(&file).unwrap());

        let py_file = tmp.path().join("hello.py");
        fs::write(
            &py_file,
            "# steps:\n#   4. 删除顶部的 \"# I AM NOT DONE\"\nimport rclpy",
        )
        .unwrap();
        assert!(!Exercise::has_done_marker(&py_file).unwrap());
    }

    #[test]
    fn test_has_done_marker_xml() {
        let tmp = TempDir::new().unwrap();
        let file = tmp.path().join("robot.urdf");
        fs::write(&file, "<!-- I AM NOT DONE -->\n<robot name=\"r\"/>").unwrap();
        assert!(Exercise::has_done_marker(&file).unwrap());
    }

    #[test]
    fn test_source_file_finds_urdf() {
        let tmp = TempDir::new().unwrap();
        let urdf_dir = tmp.path().join("05_urdf/first/urdf");
        fs::create_dir_all(&urdf_dir).unwrap();
        fs::write(urdf_dir.join("robot.urdf"), "<robot/>").unwrap();

        let info = ExerciseInfo {
            name: "first".to_string(),
            dir: "05_urdf/first".to_string(),
            module: "URDF".to_string(),
            mode: ExerciseMode::Fix,
            language: Language::Urdf,
            difficulty: 1,
            estimated_minutes: 5,
            hint_count: 1,
            depends_on: vec![],
            test: true,
            hint: String::new(),
        };
        let ex = Exercise::new(info, tmp.path().to_path_buf());
        let source = ex.source_file().unwrap();
        assert!(source.to_str().unwrap().ends_with("robot.urdf"));
    }

    #[test]
    fn test_source_file_finds_xacro() {
        let tmp = TempDir::new().unwrap();
        let urdf_dir = tmp.path().join("05_urdf/xbasics/urdf");
        fs::create_dir_all(&urdf_dir).unwrap();
        fs::write(urdf_dir.join("robot.urdf.xacro"), "<robot/>").unwrap();

        let info = ExerciseInfo {
            name: "xbasics".to_string(),
            dir: "05_urdf/xbasics".to_string(),
            module: "URDF".to_string(),
            mode: ExerciseMode::Fix,
            language: Language::Xacro,
            difficulty: 1,
            estimated_minutes: 5,
            hint_count: 1,
            depends_on: vec![],
            test: true,
            hint: String::new(),
        };
        let ex = Exercise::new(info, tmp.path().to_path_buf());
        let source = ex.source_file().unwrap();
        assert!(source.to_str().unwrap().ends_with(".xacro"));
    }

    #[test]
    fn test_any_done_marker_checks_all_files() {
        let tmp = TempDir::new().unwrap();
        let src_dir = tmp.path().join("03_launch/first/src");
        let launch_dir = tmp.path().join("03_launch/first/launch");
        fs::create_dir_all(&src_dir).unwrap();
        fs::create_dir_all(&launch_dir).unwrap();
        // src file is clean (no marker)
        fs::write(src_dir.join("node.py"), "import rclpy").unwrap();
        // launch file still has marker
        fs::write(
            launch_dir.join("first.launch.py"),
            "# I AM NOT DONE\nfrom launch import LaunchDescription",
        )
        .unwrap();

        let info = ExerciseInfo {
            name: "first".to_string(),
            dir: "03_launch/first".to_string(),
            module: "Launch".to_string(),
            mode: ExerciseMode::Fix,
            language: Language::Python,
            difficulty: 1,
            estimated_minutes: 5,
            hint_count: 1,
            depends_on: vec![],
            test: true,
            hint: String::new(),
        };
        let ex = Exercise::new(info, tmp.path().to_path_buf());
        // source_file() now prefers the file with the marker (launch/first.launch.py)
        let source = ex.source_file().unwrap();
        assert!(
            source.to_str().unwrap().contains("launch"),
            "Expected launch file, got: {}",
            source.display()
        );
        // any_done_marker() also detects it
        assert!(ex.any_done_marker().unwrap());
    }

    #[test]
    fn test_source_file_falls_back_to_first_when_no_marker() {
        let tmp = TempDir::new().unwrap();
        let src_dir = tmp.path().join("03_launch/done/src");
        let launch_dir = tmp.path().join("03_launch/done/launch");
        fs::create_dir_all(&src_dir).unwrap();
        fs::create_dir_all(&launch_dir).unwrap();
        // Both files clean — no markers
        fs::write(src_dir.join("node.py"), "import rclpy").unwrap();
        fs::write(launch_dir.join("app.launch.py"), "from launch import LD").unwrap();

        let info = ExerciseInfo {
            name: "done".to_string(),
            dir: "03_launch/done".to_string(),
            module: "Launch".to_string(),
            mode: ExerciseMode::Fix,
            language: Language::Python,
            difficulty: 1,
            estimated_minutes: 5,
            hint_count: 1,
            depends_on: vec![],
            test: true,
            hint: String::new(),
        };
        let ex = Exercise::new(info, tmp.path().to_path_buf());
        // No marker → returns first candidate (src/)
        let source = ex.source_file().unwrap();
        assert!(source.to_str().unwrap().contains("src"));
    }

    #[test]
    fn test_source_file_finds_launch_py() {
        let tmp = TempDir::new().unwrap();
        let launch_dir = tmp.path().join("03_launch/first/launch");
        fs::create_dir_all(&launch_dir).unwrap();
        fs::write(launch_dir.join("first.launch.py"), "# launch").unwrap();

        let info = ExerciseInfo {
            name: "first".to_string(),
            dir: "03_launch/first".to_string(),
            module: "Launch".to_string(),
            mode: ExerciseMode::Fix,
            language: Language::Python,
            difficulty: 1,
            estimated_minutes: 5,
            hint_count: 1,
            depends_on: vec![],
            test: true,
            hint: String::new(),
        };
        let ex = Exercise::new(info, tmp.path().to_path_buf());
        let source = ex.source_file().unwrap();
        assert!(source.to_str().unwrap().contains("launch"));
        assert!(source.to_str().unwrap().ends_with(".py"));
    }
}
