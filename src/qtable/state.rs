use std::{env, sync::LazyLock};

#[derive(Clone, Copy)]
pub struct State(pub(super) usize);

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
}
