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

    pub fn source_file(&self) -> Result<PathBuf> {
        let dir = self.dir_path();

        // Language-aware search: define (subdirs, extensions) per language
        let search_plan: Vec<(PathBuf, &[&str])> = match self.info.language {
            Language::Cpp => vec![
                (dir.join("src"), &["cpp", "c"]),
            ],
            Language::C => vec![
                (dir.join("src"), &["c"]),
            ],
            Language::Python => vec![
                (dir.join("src"), &["py"]),
                (dir.join("launch"), &["py"]),
                (dir.clone(), &["py"]),
            ],
            Language::Urdf => vec![
                (dir.join("urdf"), &["urdf"]),
            ],
            Language::Xacro => vec![
                (dir.join("urdf"), &["xacro"]),
            ],
        };

        for (search_dir, extensions) in &search_plan {
            if let Some(found) = Self::find_file_with_extensions(search_dir, extensions)? {
                return Ok(found);
            }
        }

        anyhow::bail!("No source file found in exercise '{}'", self.info.name)
    }

    fn find_file_with_extensions(dir: &Path, extensions: &[&str]) -> Result<Option<PathBuf>> {
        if !dir.is_dir() {
            return Ok(None);
        }
        for entry in
            std::fs::read_dir(dir).with_context(|| format!("Failed to read {}", dir.display()))?
        {
            let entry = entry?;
            let path = entry.path();
            if let Some(ext) = path.extension() {
                if let Some(ext_str) = ext.to_str() {
                    if extensions.contains(&ext_str) {
                        return Ok(Some(path));
                    }
                }
            }
        }
        Ok(None)
    }

    pub fn has_done_marker(source_path: &Path) -> Result<bool> {
        let content = std::fs::read_to_string(source_path)
            .with_context(|| format!("Failed to read {}", source_path.display()))?;
        Ok(content.contains(DONE_MARKER)
            || content.contains(DONE_MARKER_PY)
            || content.contains(DONE_MARKER_XML))
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
