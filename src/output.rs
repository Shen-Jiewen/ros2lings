use crossterm::style::{Attribute, Color, ResetColor, SetAttribute, SetForegroundColor};
use std::io::{self, Write};

pub fn print_success(msg: &str) {
    let mut stdout = io::stdout();
    let _ = write!(
        stdout,
        "{}{}  ✓ {}{}",
        SetForegroundColor(Color::Green),
        SetAttribute(Attribute::Bold),
        msg,
        ResetColor
    );
    let _ = writeln!(stdout);
}

pub fn print_error(msg: &str) {
    let mut stderr = io::stderr();
    let _ = write!(
        stderr,
        "{}{}  ✗ {}{}",
        SetForegroundColor(Color::Red),
        SetAttribute(Attribute::Bold),
        msg,
        ResetColor
    );
    let _ = writeln!(stderr);
}

pub fn print_warning(msg: &str) {
    let mut stdout = io::stdout();
    let _ = write!(
        stdout,
        "{}{}  ⚠ {}{}",
        SetForegroundColor(Color::Yellow),
        SetAttribute(Attribute::Bold),
        msg,
        ResetColor
    );
    let _ = writeln!(stdout);
}

pub fn print_info(msg: &str) {
    let mut stdout = io::stdout();
    let _ = write!(
        stdout,
        "{}  ℹ {}{}",
        SetForegroundColor(Color::Cyan),
        msg,
        ResetColor
    );
    let _ = writeln!(stdout);
}

pub fn print_exercise_header(name: &str, module: &str, mode: &str, difficulty: u8) {
    let stars = "★".repeat(difficulty as usize) + &"☆".repeat(5 - difficulty as usize);
    println!();
    let mut stdout = io::stdout();
    let _ = write!(
        stdout,
        "{}{}── {} ({}) ──{}",
        SetForegroundColor(Color::Blue),
        SetAttribute(Attribute::Bold),
        name,
        module,
        ResetColor,
    );
    let _ = writeln!(stdout);
    println!("   Type: {}  Difficulty: {}", mode, stars);
    println!();
}

pub fn print_progress_bar(done: usize, total: usize) {
    let width = 40;
    let filled = if total > 0 { (done * width) / total } else { 0 };
    let empty = width - filled;

    let mut stdout = io::stdout();
    let _ = writeln!(
        stdout,
        "  Progress: [{}{}{}{}] {}/{} ({:.0}%)",
        SetForegroundColor(Color::Green),
        "█".repeat(filled),
        SetForegroundColor(Color::DarkGrey),
        "░".repeat(empty),
        done,
        total,
        if total > 0 {
            (done as f64 / total as f64) * 100.0
        } else {
            0.0
        }
    );
    let _ = write!(stdout, "{}", ResetColor);
}

#[allow(dead_code)]
pub fn print_welcome(message: &str) {
    let mut stdout = io::stdout();
    let _ = writeln!(
        stdout,
        "{}{}{}{}",
        SetForegroundColor(Color::Cyan),
        SetAttribute(Attribute::Bold),
        message,
        ResetColor,
    );
}
