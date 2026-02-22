use crate::exercise::Exercise;
use crate::ros2_env::Ros2Env;
use anyhow::{Context, Result};
use std::collections::hash_map::DefaultHasher;
use std::hash::{Hash, Hasher};
use std::path::PathBuf;
use std::process::{Command, Output};

#[derive(Debug, PartialEq)]
pub enum VerifyResult {
    NotReady,
    BuildFailed(String),
    TestFailed(String),
    Success,
}

/// Single-quote a value for safe interpolation into `bash -c` commands.
/// Replaces any embedded `'` with `'\''` (end quote, escaped quote, restart quote).
fn shell_quote(s: &str) -> String {
    format!("'{}'", s.replace('\'', "'\\''"))
}

pub struct VerifyPipeline {
    ros2_env: Ros2Env,
    project_root: PathBuf,
    build_base: PathBuf,
    install_base: PathBuf,
}

impl VerifyPipeline {
    pub fn new(ros2_env: Ros2Env, project_root: PathBuf) -> Self {
        let mut hasher = DefaultHasher::new();
        project_root.hash(&mut hasher);
        let hash = format!("{:016x}", hasher.finish());
        let base = PathBuf::from(format!("/tmp/ros2lings_{hash}"));
        Self {
            ros2_env,
            build_base: base.join("build"),
            install_base: base.join("install"),
            project_root,
        }
    }

    pub fn verify(&self, exercise: &Exercise) -> Result<VerifyResult> {
        if exercise.any_done_marker()? {
            return Ok(VerifyResult::NotReady);
        }

        let build_result = self.colcon_build(exercise)?;
        if !build_result.status.success() {
            let stderr = String::from_utf8_lossy(&build_result.stderr).to_string();
            let stdout = String::from_utf8_lossy(&build_result.stdout).to_string();
            return Ok(VerifyResult::BuildFailed(format!("{}\n{}", stdout, stderr)));
        }

        if exercise.info.test {
            let test_result = self.colcon_test(exercise)?;
            if !test_result.status.success() {
                let stderr = String::from_utf8_lossy(&test_result.stderr).to_string();
                let stdout = String::from_utf8_lossy(&test_result.stdout).to_string();
                return Ok(VerifyResult::TestFailed(format!("{}\n{}", stdout, stderr)));
            }

            let test_result_output = self.colcon_test_result(exercise)?;
            if !test_result_output.status.success() {
                let stdout = String::from_utf8_lossy(&test_result_output.stdout).to_string();
                return Ok(VerifyResult::TestFailed(stdout));
            }
        }

        Ok(VerifyResult::Success)
    }

    fn colcon_build(&self, exercise: &Exercise) -> Result<Output> {
        let pkg = exercise.info.package_name();
        let exercise_path = exercise.dir_path();

        // Build --paths: always include the exercise; also include ros2lings_interfaces
        // so exercises depending on custom msgs/srvs/actions can resolve their deps.
        let interfaces_dir = self.project_root.join("exercises/ros2lings_interfaces");
        let paths = if interfaces_dir.is_dir() {
            format!(
                "{} {}",
                shell_quote(&exercise_path.display().to_string()),
                shell_quote(&interfaces_dir.display().to_string()),
            )
        } else {
            shell_quote(&exercise_path.display().to_string())
        };

        let build_base = shell_quote(&self.build_base.display().to_string());
        let install_base = shell_quote(&self.install_base.display().to_string());
        let pkg = shell_quote(&pkg);

        let shell_cmd = format!(
            "{}colcon build \
             --paths {paths} \
             --packages-up-to {pkg} \
             --build-base {build_base} \
             --install-base {install_base} \
             --event-handlers console_direct+ \
             --cmake-args -DCMAKE_BUILD_TYPE=Debug",
            self.ros2_env.shell_prefix(),
        );

        Command::new("bash")
            .args(["-c", &shell_cmd])
            .current_dir(&self.project_root)
            .output()
            .with_context(|| format!("Failed to run colcon build for {}", exercise.info.package_name()))
    }

    fn colcon_test(&self, exercise: &Exercise) -> Result<Output> {
        let pkg = shell_quote(&exercise.info.package_name());
        let build_base = shell_quote(&self.build_base.display().to_string());
        let install_base = shell_quote(&self.install_base.display().to_string());

        let shell_cmd = format!(
            "{}colcon test \
             --packages-select {pkg} \
             --build-base {build_base} \
             --install-base {install_base} \
             --event-handlers console_direct+",
            self.ros2_env.shell_prefix(),
        );

        Command::new("bash")
            .args(["-c", &shell_cmd])
            .current_dir(&self.project_root)
            .output()
            .with_context(|| format!("Failed to run colcon test for {}", exercise.info.package_name()))
    }

    fn colcon_test_result(&self, exercise: &Exercise) -> Result<Output> {
        let pkg = exercise.info.package_name();
        let test_base = self.build_base.join(&pkg);
        let test_base = shell_quote(&test_base.display().to_string());

        let shell_cmd = format!(
            "{}colcon test-result \
             --test-result-base {test_base}",
            self.ros2_env.shell_prefix(),
        );

        Command::new("bash")
            .args(["-c", &shell_cmd])
            .output()
            .with_context(|| format!("Failed to get test results for {}", pkg))
    }
}

pub fn format_build_error(raw_output: &str) -> String {
    let mut errors = Vec::new();

    for line in raw_output.lines() {
        if line.contains(": error:")
            || line.contains(": fatal error:")
            || line.starts_with("CMake Error")
        {
            errors.push(line.trim().to_string());
        }
    }

    if errors.is_empty() {
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

    #[test]
    fn test_shell_quote_simple() {
        assert_eq!(shell_quote("hello"), "'hello'");
    }

    #[test]
    fn test_shell_quote_with_single_quote() {
        assert_eq!(shell_quote("it's"), "'it'\\''s'");
    }

    #[test]
    fn test_shell_quote_with_spaces() {
        assert_eq!(shell_quote("/path/with spaces/dir"), "'/path/with spaces/dir'");
    }

    #[test]
    fn test_build_base_is_unique_per_project() {
        let env = Ros2Env {
            distro: "humble".to_string(),
            setup_bash: PathBuf::from("/opt/ros/humble/setup.bash"),
        };
        let p1 = VerifyPipeline::new(env, PathBuf::from("/home/a/project1"));

        let env2 = Ros2Env {
            distro: "humble".to_string(),
            setup_bash: PathBuf::from("/opt/ros/humble/setup.bash"),
        };
        let p2 = VerifyPipeline::new(env2, PathBuf::from("/home/a/project2"));

        assert_ne!(p1.build_base, p2.build_base);
        assert_ne!(p1.install_base, p2.install_base);
    }
}
