use crate::app_state::AppState;
use crate::exercise::Exercise;
use crate::output;
use crate::ros2_env::Ros2Env;
use crate::verify::{VerifyPipeline, VerifyResult};
use anyhow::Result;
use crossterm::event::{self, Event, KeyCode, KeyEvent};
use notify_debouncer_mini::{new_debouncer, DebouncedEventKind};
use std::path::{Path, PathBuf};
use std::sync::mpsc;
use std::time::{Duration, Instant};

enum WatchEvent {
    FileChanged(PathBuf),
    Key(KeyEvent),
}

pub fn run_watch(project_root: &Path, state: &mut AppState) -> Result<()> {
    let ros2_env = Ros2Env::detect()?;
    let pipeline = VerifyPipeline::new(ros2_env, project_root.to_path_buf());
    let exercises_root = project_root.join("exercises");

    println!();
    output::print_info("ROS2lings — Watch Mode");
    output::print_info(
        "Watching for file changes... (press 'q' to quit, 'h' for hint, 'l' to list, 'n' to skip)",
    );
    println!();

    show_current_exercise(state, &exercises_root);

    let (tx, rx) = mpsc::channel();
    let tx_file = tx.clone();

    let mut debouncer = new_debouncer(
        Duration::from_millis(300),
        move |res: notify_debouncer_mini::DebounceEventResult| {
            if let Ok(events) = res {
                for event in events {
                    if event.kind == DebouncedEventKind::Any {
                        let _ = tx_file.send(WatchEvent::FileChanged(event.path));
                    }
                }
            }
        },
    )?;

    debouncer.watcher().watch(
        &exercises_root,
        notify_debouncer_mini::notify::RecursiveMode::Recursive,
    )?;

    let tx_key = tx.clone();
    std::thread::spawn(move || loop {
        if let Ok(Event::Key(key)) = event::read() {
            let _ = tx_key.send(WatchEvent::Key(key));
        }
    });

    let mut last_verify = Instant::now() - Duration::from_secs(10);

    loop {
        match rx.recv() {
            Ok(WatchEvent::FileChanged(path)) => {
                // Throttle: ignore events within 1s of the last verify start
                if last_verify.elapsed() < Duration::from_secs(1) {
                    continue;
                }

                // Skip build artifacts and cache directories
                if path.components().any(|c| {
                    let s = c.as_os_str().to_string_lossy();
                    s == "__pycache__" || s == "build" || s == ".cache" || s == "log"
                }) {
                    continue;
                }

                let current = state.current_exercise();
                let current_dir = exercises_root.join(&current.dir);

                if path.starts_with(&current_dir) {
                    if let Some(ext) = path.extension() {
                        if let Some("cpp" | "py" | "c" | "h" | "hpp" | "urdf" | "xacro" | "xml" | "yaml" | "yml") = ext.to_str() {
                            println!();
                            output::print_info(&format!(
                                "File changed: {}",
                                path.file_name().unwrap_or_default().to_string_lossy()
                            ));
                            last_verify = Instant::now();
                            run_verify(&pipeline, state, &exercises_root)?;

                            // Drain any queued file events accumulated during verification
                            while rx.try_recv().is_ok() {}
                        }
                    }
                }
            }
            Ok(WatchEvent::Key(key)) => match key.code {
                KeyCode::Char('q') => {
                    println!();
                    output::print_info("Goodbye! Keep learning ROS2!");
                    break;
                }
                KeyCode::Char('h') => {
                    let exercise = state.current_exercise().clone();
                    let hint_level = state.next_hint_level(&exercise.name);
                    match crate::hint::show_hint(&exercise, &exercises_root, hint_level) {
                        Ok((level, content)) => {
                            println!();
                            println!("{}", content);
                            let remaining = exercise.hint_count.saturating_sub(level);
                            if remaining > 0 {
                                output::print_info(&format!(
                                    "Press 'h' again for more hints ({} remaining)",
                                    remaining
                                ));
                            }
                            let _ = state.save();
                        }
                        Err(e) => output::print_error(&format!("Hint error: {}", e)),
                    }
                }
                KeyCode::Char('l') => {
                    show_current_exercise(state, &exercises_root);
                }
                KeyCode::Char('n') => {
                    output::print_info("Skipping to next exercise...");
                    state.done_current_exercise()?;
                    show_current_exercise(state, &exercises_root);
                }
                _ => {}
            },
            Err(_) => break,
        }
    }

    Ok(())
}

fn show_current_exercise(state: &AppState, exercises_root: &Path) {
    let ex = state.current_exercise();
    output::print_exercise_header(
        &ex.name,
        &ex.module,
        &format!("{:?}", ex.mode).to_lowercase(),
        ex.difficulty,
    );
    let (done, total) = state.progress();
    output::print_progress_bar(done, total);
    println!();
    let exercise = Exercise::new(ex.clone(), exercises_root.to_path_buf());
    let path_display = match exercise.source_file() {
        Ok(path) => format!("{}", path.display()),
        Err(_) => format!("{}", exercises_root.join(&ex.dir).display()),
    };
    output::print_info(&format!("Open the exercise: {}", path_display));
    println!();
}

fn run_verify(
    pipeline: &VerifyPipeline,
    state: &mut AppState,
    exercises_root: &Path,
) -> Result<()> {
    let info = state.current_exercise().clone();
    let exercise = Exercise::new(info.clone(), exercises_root.to_path_buf());

    output::print_info("Compiling and testing...");

    match pipeline.verify(&exercise)? {
        VerifyResult::NotReady => {
            output::print_warning("Remove the 'I AM NOT DONE' marker when you're ready to verify.");
        }
        VerifyResult::BuildFailed(out) => {
            output::print_error("Build failed!");
            let formatted = crate::verify::format_build_error(&out);
            println!("{}", formatted);
        }
        VerifyResult::TestFailed(out) => {
            output::print_error("Tests failed!");
            println!("{}", out);
        }
        VerifyResult::Success => {
            output::print_success(&format!("Exercise '{}' passed!", info.name));
            state.done_current_exercise()?;
            if state.all_done() {
                output::print_success("Congratulations! You've completed all exercises!");
            } else {
                show_current_exercise(state, exercises_root);
            }
        }
    }

    Ok(())
}
