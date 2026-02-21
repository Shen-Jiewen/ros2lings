mod cmd;
mod info_file;
mod app_state;
mod exercise;

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
