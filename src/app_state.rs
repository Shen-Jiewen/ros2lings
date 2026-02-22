use crate::info_file::ExerciseInfo;
use anyhow::{Context, Result};
use std::collections::{HashMap, HashSet};
use std::path::{Path, PathBuf};

const STATE_FILE: &str = ".ros2lings-state.txt";

pub struct AppState {
    pub exercises: Vec<ExerciseInfo>,
    pub current_index: usize,
    pub done: HashSet<String>,
    hint_levels: HashMap<String, u32>,
    state_file: PathBuf,
}

impl AppState {
    pub fn load(project_root: &Path, exercises: Vec<ExerciseInfo>) -> Result<Self> {
        let state_file = project_root.join(STATE_FILE);
        let mut state = AppState {
            exercises,
            current_index: 0,
            done: HashSet::new(),
            hint_levels: HashMap::new(),
            state_file,
        };

        if state.state_file.exists() {
            state.read_state()?;
        }

        // Prune done and hint entries that don't match any known exercise name
        let known_names: HashSet<&str> = state.exercises.iter().map(|e| e.name.as_str()).collect();
        state.done.retain(|name| known_names.contains(name.as_str()));
        state
            .hint_levels
            .retain(|name, _| known_names.contains(name.as_str()));

        Ok(state)
    }

    fn read_state(&mut self) -> Result<()> {
        let content = std::fs::read_to_string(&self.state_file)
            .with_context(|| "Failed to read state file")?;

        for line in content.lines() {
            let line = line.trim();
            if line.is_empty() || line.starts_with('#') {
                continue;
            }
            if let Some(value) = line.strip_prefix("current=") {
                if let Some(idx) = self.exercises.iter().position(|e| e.name == value) {
                    self.current_index = idx;
                }
            } else if let Some(value) = line.strip_prefix("done=") {
                for name in value.split(',') {
                    let name = name.trim();
                    if !name.is_empty() {
                        self.done.insert(name.to_string());
                    }
                }
            } else if let Some(value) = line.strip_prefix("hints=") {
                for pair in value.split(',') {
                    let pair = pair.trim();
                    if let Some((name, level_str)) = pair.split_once(':') {
                        if let Ok(level) = level_str.parse::<u32>() {
                            self.hint_levels.insert(name.to_string(), level);
                        }
                    }
                }
            }
        }

        Ok(())
    }

    pub fn save(&self) -> Result<()> {
        let mut done_list: Vec<&str> = self.done.iter().map(|s| s.as_str()).collect();
        done_list.sort();
        let current_name = &self.exercises[self.current_index].name;
        let mut hints_pairs: Vec<String> = self
            .hint_levels
            .iter()
            .map(|(k, v)| format!("{}:{}", k, v))
            .collect();
        hints_pairs.sort();
        let content = format!(
            "# ROS2lings learning progress — do not edit manually\ncurrent={}\ndone={}\nhints={}\n",
            current_name,
            done_list.join(","),
            hints_pairs.join(","),
        );
        std::fs::write(&self.state_file, content).with_context(|| "Failed to write state file")?;
        Ok(())
    }

    pub fn done_current_exercise(&mut self) -> Result<()> {
        let name = self.exercises[self.current_index].name.clone();
        self.done.insert(name);

        for i in 0..self.exercises.len() {
            let idx = (self.current_index + 1 + i) % self.exercises.len();
            if !self.done.contains(&self.exercises[idx].name) {
                self.current_index = idx;
                self.save()?;
                return Ok(());
            }
        }

        self.save()?;
        Ok(())
    }

    pub fn current_exercise(&self) -> &ExerciseInfo {
        &self.exercises[self.current_index]
    }

    pub fn is_done(&self, name: &str) -> bool {
        self.done.contains(name)
    }

    pub fn all_done(&self) -> bool {
        self.done.len() >= self.exercises.len()
    }

    pub fn progress(&self) -> (usize, usize) {
        (self.done.len(), self.exercises.len())
    }

    pub fn find_exercise(&self, name: &str) -> Option<usize> {
        self.exercises.iter().position(|e| e.name == name)
    }

    /// Returns the current hint level for an exercise, then increments for next call.
    pub fn next_hint_level(&mut self, exercise_name: &str) -> u32 {
        let level = self.hint_levels.get(exercise_name).copied().unwrap_or(0);
        self.hint_levels
            .insert(exercise_name.to_string(), level + 1);
        level
    }

    #[cfg(test)]
    pub fn current_hint_level(&self, exercise_name: &str) -> u32 {
        self.hint_levels.get(exercise_name).copied().unwrap_or(0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::info_file::{ExerciseMode, Language};
    use tempfile::TempDir;

    fn make_exercises() -> Vec<ExerciseInfo> {
        vec![
            ExerciseInfo {
                name: "ex1".to_string(),
                dir: "m01/ex1".to_string(),
                module: "M1".to_string(),
                mode: ExerciseMode::Fix,
                language: Language::Cpp,
                difficulty: 1,
                estimated_minutes: 5,
                hint_count: 1,
                depends_on: vec![],
                test: true,
                hint: "h".to_string(),
            },
            ExerciseInfo {
                name: "ex2".to_string(),
                dir: "m01/ex2".to_string(),
                module: "M1".to_string(),
                mode: ExerciseMode::Fix,
                language: Language::Cpp,
                difficulty: 1,
                estimated_minutes: 5,
                hint_count: 1,
                depends_on: vec![],
                test: true,
                hint: "h".to_string(),
            },
            ExerciseInfo {
                name: "ex3".to_string(),
                dir: "m01/ex3".to_string(),
                module: "M1".to_string(),
                mode: ExerciseMode::Fix,
                language: Language::Cpp,
                difficulty: 1,
                estimated_minutes: 5,
                hint_count: 1,
                depends_on: vec![],
                test: true,
                hint: "h".to_string(),
            },
        ]
    }

    #[test]
    fn test_new_state_starts_at_first_exercise() {
        let tmp = TempDir::new().unwrap();
        let state = AppState::load(tmp.path(), make_exercises()).unwrap();
        assert_eq!(state.current_index, 0);
        assert_eq!(state.current_exercise().name, "ex1");
        assert!(state.done.is_empty());
    }

    #[test]
    fn test_save_and_reload() {
        let tmp = TempDir::new().unwrap();
        {
            let mut state = AppState::load(tmp.path(), make_exercises()).unwrap();
            state.done.insert("ex1".to_string());
            state.current_index = 1;
            state.save().unwrap();
        }
        {
            let state = AppState::load(tmp.path(), make_exercises()).unwrap();
            assert_eq!(state.current_index, 1);
            assert!(state.is_done("ex1"));
            assert!(!state.is_done("ex2"));
        }
    }

    #[test]
    fn test_done_current_advances() {
        let tmp = TempDir::new().unwrap();
        let mut state = AppState::load(tmp.path(), make_exercises()).unwrap();
        assert_eq!(state.current_exercise().name, "ex1");
        state.done_current_exercise().unwrap();
        assert_eq!(state.current_exercise().name, "ex2");
        assert!(state.is_done("ex1"));
    }

    #[test]
    fn test_done_skips_completed() {
        let tmp = TempDir::new().unwrap();
        let mut state = AppState::load(tmp.path(), make_exercises()).unwrap();
        state.done.insert("ex2".to_string());
        state.done_current_exercise().unwrap();
        assert_eq!(state.current_exercise().name, "ex3");
    }

    #[test]
    fn test_all_done() {
        let tmp = TempDir::new().unwrap();
        let mut state = AppState::load(tmp.path(), make_exercises()).unwrap();
        assert!(!state.all_done());
        state.done.insert("ex1".to_string());
        state.done.insert("ex2".to_string());
        state.done.insert("ex3".to_string());
        assert!(state.all_done());
    }

    #[test]
    fn test_progress() {
        let tmp = TempDir::new().unwrap();
        let mut state = AppState::load(tmp.path(), make_exercises()).unwrap();
        assert_eq!(state.progress(), (0, 3));
        state.done.insert("ex1".to_string());
        assert_eq!(state.progress(), (1, 3));
    }

    #[test]
    fn test_find_exercise() {
        let tmp = TempDir::new().unwrap();
        let state = AppState::load(tmp.path(), make_exercises()).unwrap();
        assert_eq!(state.find_exercise("ex2"), Some(1));
        assert_eq!(state.find_exercise("nonexistent"), None);
    }

    #[test]
    fn test_unknown_done_entries_pruned() {
        let tmp = TempDir::new().unwrap();
        // Manually write a state file with an unknown exercise in done
        let state_content = "current=ex1\ndone=ex1,ghost_exercise,ex2\n";
        std::fs::write(tmp.path().join(".ros2lings-state.txt"), state_content).unwrap();

        let state = AppState::load(tmp.path(), make_exercises()).unwrap();
        // ghost_exercise should be pruned
        assert!(!state.is_done("ghost_exercise"));
        assert!(state.is_done("ex1"));
        assert!(state.is_done("ex2"));
        // Progress should only count valid exercises
        assert_eq!(state.progress(), (2, 3));
    }

    #[test]
    fn test_hint_level_progression() {
        let tmp = TempDir::new().unwrap();
        let mut state = AppState::load(tmp.path(), make_exercises()).unwrap();
        assert_eq!(state.current_hint_level("ex1"), 0);
        assert_eq!(state.next_hint_level("ex1"), 0);
        assert_eq!(state.current_hint_level("ex1"), 1);
        assert_eq!(state.next_hint_level("ex1"), 1);
        assert_eq!(state.current_hint_level("ex1"), 2);
    }

    #[test]
    fn test_hint_levels_persist() {
        let tmp = TempDir::new().unwrap();
        {
            let mut state = AppState::load(tmp.path(), make_exercises()).unwrap();
            state.next_hint_level("ex1"); // 0 -> stored as 1
            state.next_hint_level("ex1"); // 1 -> stored as 2
            state.next_hint_level("ex2"); // 0 -> stored as 1
            state.save().unwrap();
        }
        {
            let state = AppState::load(tmp.path(), make_exercises()).unwrap();
            assert_eq!(state.current_hint_level("ex1"), 2);
            assert_eq!(state.current_hint_level("ex2"), 1);
            assert_eq!(state.current_hint_level("ex3"), 0);
        }
    }

    #[test]
    fn test_stale_hint_levels_pruned() {
        let tmp = TempDir::new().unwrap();
        // Write a state file with hint levels for a ghost exercise
        let state_content = "current=ex1\ndone=\nhints=ex1:2,ghost:5\n";
        std::fs::write(tmp.path().join(".ros2lings-state.txt"), state_content).unwrap();

        let state = AppState::load(tmp.path(), make_exercises()).unwrap();
        assert_eq!(state.current_hint_level("ex1"), 2);
        // ghost should be pruned
        assert_eq!(state.current_hint_level("ghost"), 0);
    }
}
