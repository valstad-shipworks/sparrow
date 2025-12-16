use jagua_rs::Instant;
use std::{
    sync::{Arc, atomic::AtomicBool},
    time::Duration,
};

/// Generic trait for any struct that can determine if the optimization process should terminate.
pub trait Terminator: Clone {
    /// Checks if the termination condition is met
    fn should_terminate(&self) -> bool;
}

#[derive(Debug, Clone)]
pub struct TimedTerminator {
    timeout: Instant,
}

impl TimedTerminator {
    pub fn new_duration(timeout: Duration) -> Self {
        Self {
            timeout: Instant::now() + timeout,
        }
    }

    pub fn new_instant(timeout: Instant) -> Self {
        Self { timeout }
    }
}

impl Terminator for TimedTerminator {
    fn should_terminate(&self) -> bool {
        Instant::now() > self.timeout
    }
}

#[derive(Debug, Clone)]
pub struct FlagTerminator {
    flag: Arc<AtomicBool>,
}

impl FlagTerminator {
    pub fn new() -> Self {
        Self { flag: Arc::new(AtomicBool::new(false)) }
    }

    pub fn of(flag: Arc<AtomicBool>) -> Self {
        Self { flag }
    }
}

impl Terminator for FlagTerminator {
    fn should_terminate(&self) -> bool {
        self.flag.load(std::sync::atomic::Ordering::Relaxed)
    }
}

impl Default for FlagTerminator {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Debug, Clone)]
pub struct CombinedTerminator<T1: Terminator, T2: Terminator> {
    term1: T1,
    term2: T2,
}

impl<T1: Terminator, T2: Terminator> CombinedTerminator<T1, T2> {
    pub fn new(term1: T1, term2: T2) -> Self {
        Self { term1, term2 }
    }
}

impl<T1: Terminator, T2: Terminator> Terminator for CombinedTerminator<T1, T2> {
    fn should_terminate(&self) -> bool {
        self.term1.should_terminate() || self.term2.should_terminate()
    }
}