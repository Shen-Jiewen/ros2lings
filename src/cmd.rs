use crate::app_state::AppState;
use crate::exercise::Exercise;
use crate::info_file::InfoFile;
use crate::output;
use crate::ros2_env::Ros2Env;
use crate::verify::{self, VerifyPipeline, VerifyResult};
use crate::{Cli, Commands};
use anyhow::{Context, Result};
use std::path::Path;

pub fn run(cli: Cli) -> Result<()> {
    let project_root = std::env::current_dir().context("Failed to get current directory")?;

    let info_path = project_root.join("info.toml");
    let info = InfoFile::parse(&info_path)?;

    let mut state = AppState::load(&project_root, info.exercises)?;

    match cli.command {
        None => cmd_watch(&project_root, &mut state),
        Some(Commands::List) => cmd_list(&state),
        Some(Commands::Hint) => cmd_hint(&project_root, &mut state),
        Some(Commands::Verify { name }) => cmd_verify(&project_root, &mut state, name.as_deref()),
        Some(Commands::Reset { name }) => cmd_reset(&project_root, &state, &name),
        Some(Commands::Explain { name }) => cmd_explain(&project_root, &state, name.as_deref()),
        Some(Commands::Graph) => cmd_graph(&state),
    }
}

fn cmd_watch(project_root: &Path, state: &mut AppState) -> Result<()> {
    crate::watch::run_watch(project_root, state)
}

fn cmd_list(state: &AppState) -> Result<()> {
    let mut current_module = String::new();

    for (i, ex) in state.exercises.iter().enumerate() {
        if ex.module != current_module {
            current_module = ex.module.clone();
            println!("\n  ── {}", current_module);
        }

        let status = if state.is_done(&ex.name) {
            "\x1b[32m✓\x1b[0m"
        } else if i == state.current_index {
            "\x1b[33m❯\x1b[0m"
        } else if !state.deps_satisfied(ex) {
            "\x1b[90m🔒\x1b[0m"
        } else {
            "\x1b[90m✗\x1b[0m"
        };

        let lang = match ex.language {
            crate::info_file::Language::Cpp => "C++ ",
            crate::info_file::Language::Python => "Py  ",
            crate::info_file::Language::C => "C   ",
            crate::info_file::Language::Urdf => "URDF",
            crate::info_file::Language::Xacro => "Xacr",
        };

        println!(
            "  {} {:30} [{:3}] {:10} ~{}min",
            status,
            ex.name,
            lang,
            format!("{:?}", ex.mode).to_lowercase(),
            ex.estimated_minutes,
        );
    }

    let (done, total) = state.progress();
    println!();
    output::print_progress_bar(done, total);
    println!();

    Ok(())
}

fn cmd_hint(project_root: &Path, state: &mut AppState) -> Result<()> {
    let exercise = state.current_exercise().clone();
    let exercises_root = project_root.join("exercises");

    let hint_level = state.next_hint_level(&exercise.name);
    let (level, content) = crate::hint::show_hint(&exercise, &exercises_root, hint_level)?;

    output::print_exercise_header(
        &exercise.name,
        &exercise.module,
        &format!("{:?}", exercise.mode).to_lowercase(),
        exercise.difficulty,
    );
    println!("{}", content);

    let remaining = exercise.hint_count.saturating_sub(level);
    if remaining > 0 {
        output::print_info(&format!(
            "Run 'ros2lings hint' again for more detailed hints ({} remaining)",
            remaining
        ));
    }

    state.save()?;
    Ok(())
}

fn cmd_verify(project_root: &Path, state: &mut AppState, name: Option<&str>) -> Result<()> {
    let ros2_env = Ros2Env::detect()?;

    let exercise_index = match name {
        Some(n) => state
            .find_exercise(n)
            .with_context(|| format!("Exercise '{}' not found", n))?,
        None => state.current_index,
    };

    let info = state.exercises[exercise_index].clone();
    let exercises_root = project_root.join("exercises");
    let exercise = Exercise::new(info.clone(), exercises_root);

    output::print_exercise_header(
        &info.name,
        &info.module,
        &format!("{:?}", info.mode).to_lowercase(),
        info.difficulty,
    );

    let pipeline = VerifyPipeline::new(ros2_env, project_root.to_path_buf());
    let result = pipeline.verify(&exercise)?;

    match result {
        VerifyResult::NotReady => {
            output::print_warning(
                "Exercise still has the 'I AM NOT DONE' marker. Remove it when you're ready to verify.",
            );
        }
        VerifyResult::BuildFailed(output_text) => {
            output::print_error("Build failed!");
            println!();
            let formatted = verify::format_build_error(&output_text);
            println!("{}", formatted);
            println!();
            output::print_info("Try 'ros2lings hint' for help.");
        }
        VerifyResult::TestFailed(output_text) => {
            output::print_error("Tests failed!");
            println!();
            println!("{}", output_text);
            println!();
            output::print_info("Try 'ros2lings hint' for help.");
        }
        VerifyResult::Success => {
            output::print_success(&format!("Exercise '{}' passed!", info.name));
            if exercise_index == state.current_index {
                state.done_current_exercise()?;
                if state.all_done() {
                    println!();
                    output::print_success("Congratulations! You've completed all exercises!");
                } else {
                    let next = state.current_exercise();
                    output::print_info(&format!("Next up: {} ({})", next.name, next.module));
                }
            }
        }
    }

    Ok(())
}

fn cmd_reset(project_root: &Path, state: &AppState, name: &str) -> Result<()> {
    let idx = state
        .find_exercise(name)
        .with_context(|| format!("Exercise '{}' not found", name))?;
    let info = &state.exercises[idx];

    let exercise_rel = Path::new("exercises").join(&info.dir);

    // Phase 1: Restore all tracked files to their original (committed) state.
    // This reverts the student's changes so they can start the exercise fresh.
    let checkout = std::process::Command::new("git")
        .args(["checkout", "HEAD", "--"])
        .arg(exercise_rel.to_string_lossy().as_ref())
        .current_dir(project_root)
        .output();

    match checkout {
        Ok(output) if output.status.success() => {}
        Ok(output) => {
            let stderr = String::from_utf8_lossy(&output.stderr);
            anyhow::bail!("git checkout failed: {}", stderr.trim());
        }
        Err(e) => {
            anyhow::bail!("Failed to run git (is it installed?): {}", e);
        }
    }

    // Phase 2: Remove untracked files (student-created files, build artifacts).
    match std::process::Command::new("git")
        .args(["clean", "-fd", "--"])
        .arg(exercise_rel.to_string_lossy().as_ref())
        .current_dir(project_root)
        .output()
    {
        Ok(output) if !output.status.success() => {
            let stderr = String::from_utf8_lossy(&output.stderr);
            output::print_warning(&format!("git clean failed: {}", stderr.trim()));
        }
        Err(e) => {
            output::print_warning(&format!("Failed to run git clean: {}", e));
        }
        _ => {}
    }

    output::print_success(&format!("Exercise '{}' has been reset.", name));
    Ok(())
}

#[cfg(test)]
fn copy_dir_recursive(src: &Path, dest: &Path) -> Result<()> {
    if !dest.exists() {
        std::fs::create_dir_all(dest)
            .with_context(|| format!("Failed to create {}", dest.display()))?;
    }
    for entry in
        std::fs::read_dir(src).with_context(|| format!("Failed to read {}", src.display()))?
    {
        let entry = entry?;
        let src_path = entry.path();
        let dest_path = dest.join(entry.file_name());
        if src_path.is_dir() {
            copy_dir_recursive(&src_path, &dest_path)?;
        } else {
            std::fs::copy(&src_path, &dest_path)
                .with_context(|| format!("Failed to reset {}", dest_path.display()))?;
        }
    }
    Ok(())
}

fn cmd_explain(project_root: &Path, state: &AppState, name: Option<&str>) -> Result<()> {
    let exercise = match name {
        Some(n) => {
            let idx = state
                .find_exercise(n)
                .with_context(|| format!("Exercise '{}' not found", n))?;
            &state.exercises[idx]
        }
        None => state.current_exercise(),
    };

    let exercises_root = project_root.join("exercises");
    let content = crate::explain::get_explanation(exercise, &exercises_root)?;

    let skin = termimad::MadSkin::default();
    skin.print_text(&content);

    Ok(())
}

fn cmd_graph(state: &AppState) -> Result<()> {
    let (done, total) = state.progress();

    println!();
    println!("  ROS2lings Learning Progress");
    println!("  ═══════════════════════════");
    println!();
    output::print_progress_bar(done, total);
    println!();

    let mut modules: Vec<(String, usize, usize)> = Vec::new();
    for ex in &state.exercises {
        if let Some(m) = modules.last_mut() {
            if m.0 == ex.module {
                m.1 += 1;
                if state.is_done(&ex.name) {
                    m.2 += 1;
                }
                continue;
            }
        }
        let is_done = if state.is_done(&ex.name) { 1 } else { 0 };
        modules.push((ex.module.clone(), 1, is_done));
    }

    for (module, total_count, done_count) in &modules {
        let pct = if *total_count > 0 {
            (*done_count as f64 / *total_count as f64) * 100.0
        } else {
            0.0
        };
        let bar_len = 20;
        let filled = if *total_count > 0 {
            (*done_count * bar_len) / *total_count
        } else {
            0
        };
        let empty = bar_len - filled;
        println!(
            "  {:25} [{}{}] {}/{} ({:.0}%)",
            module,
            "█".repeat(filled),
            "░".repeat(empty),
            done_count,
            total_count,
            pct,
        );
    }
    println!();

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;
    use tempfile::TempDir;

    #[test]
    fn test_copy_dir_recursive() {
        let tmp = TempDir::new().unwrap();
        let src = tmp.path().join("src");
        let dest = tmp.path().join("dest");
        fs::create_dir_all(src.join("sub")).unwrap();
        fs::write(src.join("a.txt"), "hello").unwrap();
        fs::write(src.join("sub/b.txt"), "world").unwrap();

        copy_dir_recursive(&src, &dest).unwrap();

        assert_eq!(fs::read_to_string(dest.join("a.txt")).unwrap(), "hello");
        assert_eq!(fs::read_to_string(dest.join("sub/b.txt")).unwrap(), "world");
    }

    #[test]
    fn test_copy_dir_recursive_overwrites_existing() {
        let tmp = TempDir::new().unwrap();
        let src = tmp.path().join("src");
        let dest = tmp.path().join("dest");
        fs::create_dir_all(&src).unwrap();
        fs::create_dir_all(&dest).unwrap();
        fs::write(src.join("f.txt"), "new").unwrap();
        fs::write(dest.join("f.txt"), "old").unwrap();

        copy_dir_recursive(&src, &dest).unwrap();

        assert_eq!(fs::read_to_string(dest.join("f.txt")).unwrap(), "new");
    }
}
