use std::{env, sync::LazyLock};

#[derive(Clone, Copy)]
pub struct State(usize);

impl std::ops::Deref for State {
    type Target = usize;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl State {
    pub fn size() -> usize {
        const DEFAULT_SIZE: usize = 15;
        static SIZE: LazyLock<usize> = LazyLock::new(|| {
            env::var("STATE_SIZE")
                .unwrap_or_else(|_| DEFAULT_SIZE.to_string())
                .parse::<usize>()
                .unwrap_or(DEFAULT_SIZE)
        });
        *SIZE
    }

    pub fn new(index: usize) -> Option<Self> {
        (index < Self::size()).then_some(Self(index))
    }
}
