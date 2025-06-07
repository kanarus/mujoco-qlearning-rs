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
        &self.qvalues[*state]
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

pub trait Strategy {
    fn determine(qtable: &QTable, state: State) -> Action;
}

pub struct Update {
    pub state: State,
    pub action: Action,
    pub reward: f64,
    pub next_state: State,
}

impl QTable {
    pub fn next_action<S: Strategy>(&self, state: State) -> Action {
        S::determine(self, state)
    }

    pub fn update(&mut self, Update {
        state,
        action,
        reward,
        next_state,
    }: Update) {
        let current_qvalue = self.qvalues[*state][*action];
        let next_max_qvalue = self.qvalues[*next_state].iter().max().unwrap().clone();

        self.qvalues[*state][*action] = QValue::new(
            (1. - self.alpha()) * (*current_qvalue)
            + self.alpha() * (reward + self.gamma() * (*next_max_qvalue)),
        ).expect("Invalid Q-value creation");
    }
}

pub mod strategy {
    use super::{Strategy, Action, QTable, State};
    use rand::{Rng, distr::{weighted::WeightedIndex}};

    pub struct Explore;
    impl Strategy for Explore {
        fn determine(qtable: &QTable, state: State) -> Action {
            let max_action_qvalue = qtable[state].iter().max().unwrap();
            let candidates = qtable[state]
                .iter()
                .enumerate()
                .filter(|(_, qvalue)| *qvalue == max_action_qvalue)
                .map(|(index, _)| Action::new(index).unwrap())
                .collect::<Vec<_>>();
            candidates[rand::rng().random_range(0..candidates.len())]
        }
    }

    pub struct SoftMax;
    impl Strategy for SoftMax {
        fn determine(qtable: &QTable, state: State) -> Action {
            let qvalues = &qtable[state];
            let max_action_qvalue = qvalues.iter().max().unwrap();
            let softmax_probabilities = {
                let exp = qvalues.iter()
                    .map(|q| (**q - **max_action_qvalue).exp())
                    .collect::<Vec<_>>();
                let exp_sum = exp.iter().sum::<f64>();
                exp.iter().map(|&e| e / exp_sum).collect::<Vec<_>>()
            };
            let selected_index = rand::rng().sample(WeightedIndex::new(&softmax_probabilities).unwrap());
            Action::new(selected_index).unwrap()
        }
    }

    pub struct EpsilonGreedy;
    impl Strategy for EpsilonGreedy {
        fn determine(qtable: &QTable, state: State) -> Action {
            if rand::rng().random_range(0.0..1.0) < qtable.epsilon() {
                Random::determine(qtable, state)
            } else {
                Explore::determine(qtable, state)
            }
        }
    }

    pub struct Random;
    impl Strategy for Random {
        fn determine(_qtable: &QTable, _state: State) -> Action {
            let selected_index = rand::rng().random_range(0..Action::size());
            Action::new(selected_index).unwrap()
        }
    }
}
