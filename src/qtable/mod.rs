mod action;
mod state;
mod value;

pub use self::{action::Action, state::State, value::QValue};

pub struct QTable {
    qvalues: Vec<Vec<QValue>>, // 2D vector for Q-values, indexed by state and action
    config: QTableConfig,
}

pub struct QTableConfig {
    pub gamma: f64, // discount factor
    pub alpha: f64, // learning rate
    pub epsilon: f64, // exploration rate
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

impl QTable {
    pub fn new() -> Self {
        Self::new_with(Default::default())
    }

    pub fn new_with(config: QTableConfig) -> Self {
        let qvalues = (0..State::size())
            .map(|_| QValue::random_collect(Action::size()))
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

    pub fn save(&self, file_path: impl AsRef<std::path::Path>) -> Result<(), std::io::Error> {
        let file = std::fs::File::create(file_path)?;
        let writer = std::io::BufWriter::new(file);
        serde_json::to_writer(writer, &self.qvalues)?;
        Ok(())
    }
}

// Doesn't implement `IndexMut` to prevent direct modification of Q-values.
// This is to ensure that Q-values are **ONLY** updated through the `update` method,
// which applies the learning algorithm correctly.
impl std::ops::Index<State> for QTable {
    type Output = [QValue];

    fn index(&self, state: State) -> &Self::Output {
        &self.qvalues[state.0]
    }
}

impl QTable {
    pub fn gamma(&self) -> f64 {
        self.config.gamma
    }
    pub fn alpha(&self) -> f64 {
        self.config.alpha
    }
    pub fn epsilon(&self) -> f64 {
        self.config.epsilon
    }
}

pub trait ActionPlan {
    fn determine(qtable: &QTable, state: State) -> Action;
}

pub struct Update {
    pub state: State,
    pub action: Action,
    pub reward: f64,
    pub next_state: State,
}

impl QTable {
    pub fn next_action<P: ActionPlan>(&self, state: State) -> Action {
        P::determine(self, state)
    }

    pub fn update(&mut self, Update {
        state,
        action,
        reward,
        next_state,
    }: Update) {
        let current_qvalue = self.qvalues[state.0][action.0];
        let next_max_qvalue = self.qvalues[next_state.0].iter().max().unwrap().clone();

        self.qvalues[state.0][action.0] = QValue::new(
            (1. - self.alpha()) * (*current_qvalue)
            + self.alpha() * (reward + self.gamma() * (*next_max_qvalue)),
        ).expect("Invalid Q-value creation");
    }
}
