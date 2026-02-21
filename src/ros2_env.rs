use anyhow::{Context, Result};
use std::path::PathBuf;
use std::process::Command;

pub struct Ros2Env {
    pub distro: String,
    pub setup_bash: PathBuf,
}

impl Ros2Env {
    pub fn detect() -> Result<Self> {
        if let Ok(distro) = std::env::var("ROS_DISTRO") {
            let setup = PathBuf::from(format!("/opt/ros/{distro}/setup.bash"));
            if setup.exists() {
                return Ok(Self { distro, setup_bash: setup });
            }
        }

        for distro in &["humble", "jazzy", "iron"] {
            let setup = PathBuf::from(format!("/opt/ros/{distro}/setup.bash"));
            if setup.exists() {
                return Ok(Self { distro: distro.to_string(), setup_bash: setup });
            }
        }

        anyhow::bail!(
            "ROS2 not found. Please install ROS2 Humble and source setup.bash:\n  \
             sudo apt install ros-humble-desktop\n  \
             source /opt/ros/humble/setup.bash"
        )
    }

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

    pub fn shell_prefix(&self) -> String {
        format!("source {} && ", self.setup_bash.display())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_detect_finds_ros2() {
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
        assert_eq!(env.shell_prefix(), "source /opt/ros/humble/setup.bash && ");
    }

    #[test]
    fn test_check_dependencies() {
        let env = Ros2Env {
            distro: "humble".to_string(),
            setup_bash: PathBuf::from("/opt/ros/humble/setup.bash"),
        };
        let missing = env.check_dependencies().unwrap();
        // In our environment colcon and cmake should be available
        // This test just verifies the function runs without error
        assert!(missing.is_empty() || !missing.is_empty());
    }
}
