use crate::exercise::Exercise;
use crate::ros2_env::Ros2Env;
use anyhow::{Context, Result};
use std::path::PathBuf;
use std::process::{Command, Output};

#[derive(Debug, PartialEq)]
pub enum VerifyResult {
    NotReady,
    BuildFailed(String),
    TestFailed(String),
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
            format!("{} {}", exercise_path.display(), interfaces_dir.display())
        } else {
            format!("{}", exercise_path.display())
        };

        let shell_cmd = format!(
            "{}colcon build \
             --paths {paths} \
             --packages-up-to {pkg} \
             --build-base /tmp/ros2lings_build \
             --install-base /tmp/ros2lings_install \
             --event-handlers console_direct+ \
             --cmake-args -DCMAKE_BUILD_TYPE=Debug",
            self.ros2_env.shell_prefix(),
        );

        Command::new("bash")
            .args(["-c", &shell_cmd])
            .current_dir(&self.project_root)
            .output()
            .with_context(|| format!("Failed to run colcon build for {}", pkg))
    }

    fn colcon_test(&self, exercise: &Exercise) -> Result<Output> {
        let pkg = exercise.info.package_name();
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
            .current_dir(&self.project_root)
            .output()
            .with_context(|| format!("Failed to run colcon test for {}", pkg))
    }

    fn colcon_test_result(&self, exercise: &Exercise) -> Result<Output> {
        let pkg = exercise.info.package_name();
        let shell_cmd = format!(
            "{}colcon test-result \
             --test-result-base /tmp/ros2lings_build/{}",
            self.ros2_env.shell_prefix(),
            pkg,
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
}
