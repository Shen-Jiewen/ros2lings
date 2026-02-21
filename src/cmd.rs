use crate::{Cli, Commands};

pub fn run(cli: Cli) -> anyhow::Result<()> {
    match cli.command {
        None => {
            println!("Entering watch mode... (not yet implemented)");
            Ok(())
        }
        Some(cmd) => match cmd {
            Commands::List => {
                println!("Listing exercises... (not yet implemented)");
                Ok(())
            }
            Commands::Hint => {
                println!("Showing hint... (not yet implemented)");
                Ok(())
            }
            Commands::Verify { name } => {
                println!("Verifying {:?}... (not yet implemented)", name);
                Ok(())
            }
            Commands::Reset { name } => {
                println!("Resetting {}... (not yet implemented)", name);
                Ok(())
            }
            Commands::Explain { name } => {
                println!("Explaining {:?}... (not yet implemented)", name);
                Ok(())
            }
            Commands::Graph => {
                println!("Showing progress... (not yet implemented)");
                Ok(())
            }
        },
    }
}
