use std::ops::Range;
use rand::{rng, Rng, distr::Uniform};

#[derive(Clone, Copy, PartialEq, PartialOrd)]
#[derive(serde::Serialize, serde::Deserialize)]
pub struct QValue(f64);

impl std::cmp::Eq for QValue {}

impl std::cmp::Ord for QValue {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        // NaN never apears in QValue because of the range constraint
        self.0.partial_cmp(&other.0).expect("Unexpected NaN in QValue comparison")
    }
}

impl std::ops::Deref for QValue {
    type Target = f64;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl QValue {
    const RANGE: Range<f64> = (-1.0)..1.0;

    pub(super) fn new(value: f64) -> Option<Self> {
        Self::RANGE.contains(&value).then_some(Self(value))
    }

    pub(super) fn random() -> Self {
        Self(rng().random_range(Self::RANGE))
    }

    pub(super) fn random_collect(size: usize) -> Vec<Self> {
        let uniform = Uniform::try_from(Self::RANGE).unwrap();
        let mut rng = rng();
        (0..size).map(|_| Self(rng.sample(uniform))).collect()
    }
}
