mod action;
mod state;
mod train;

use std::ops::Range;
use rand::{rng, Rng, distr::Uniform};

pub struct QTable {
    qvalues: Vec<Vec<QValue>>, // 2D vector for Q-values, indexed by state and action
    config: QTableConfig,
}

#[derive(Clone, Copy, serde::Deserialize)]
struct QValue(f64);

pub struct QTableConfig {
    gamma: f64, // discount factor
    alpha: f64, // learning rate
    epsilon: f64, // exploration rate
}

impl Default for QTableConfig {
    fn default() -> Self {
        Self {
            gamma: 0.99,
            alpha: 0.5,
            epsilon: 0.5,
        }
    }
}

impl QValue {
    const RANGE: Range<f64> = (-1.0)..1.0;

    fn new(value: f64) -> Option<Self> {
        if Self::RANGE.contains(&value) {
            Some(Self(value))
        } else {
            None
        }
    }

    fn random() -> Self {
        Self(rng().random_range(Self::RANGE))
    }

    fn random_collect(size: usize) -> Vec<Self> {
        let uniform = Uniform::try_from(Self::RANGE).unwrap();
        let mut rng = rng();
        (0..size).map(|_| Self(rng.sample(uniform))).collect()
    }
}

impl QTable {
    pub fn new() -> Self {
        Self::new_with(Default::default())
    }

    pub fn new_with(config: QTableConfig) -> Self {
        let state_size = state::State::size();
        let action_size = action::Action::size();

        let qvalues = (0..state_size)
            .map(|_| QValue::random_collect(action_size))
            .collect();
        
        Self { qvalues, config }
    }

    pub fn load(file_path: impl AsRef<std::path::Path>) -> Result<Self, std::io::Error> {
        Self::load_with(file_path, Default::default())
    }

    pub fn load_with(file_path: impl AsRef<std::path::Path>, config: QTableConfig) -> Result<Self, std::io::Error> {
        let file = std::fs::File::open(file_path)?;
        let reader = std::io::BufReader::new(file);
        let qvalues: Vec<Vec<QValue>> = serde_json::from_reader(reader)?;
        Ok(Self { qvalues, config })
    }
}
