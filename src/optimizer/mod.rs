use crate::config::*;
use crate::consts::LBF_SAMPLE_CONFIG;
use crate::optimizer::compress::compression_phase;
use crate::optimizer::explore::exploration_phase;
use crate::optimizer::lbf::LBFBuilder;
use crate::optimizer::separator::Separator;
use crate::util::listener::{ReportType, SolutionListener};
use crate::util::terminator::{CombinedTerminator, FlagTerminator, Terminator, TimedTerminator};
use event_listener::{Event, Listener};
use jagua_rs::probs::spp::entities::{SPInstance, SPSolution};
use rand::{RngCore, SeedableRng};
use rand_xoshiro::Xoshiro256PlusPlus;
use std::sync::{Arc, Mutex};
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;

pub mod compress;
pub mod explore;
pub mod lbf;
pub mod separator;
mod worker;

///Algorithm 11 from https://doi.org/10.48550/arXiv.2509.13329
pub fn optimize(
    instance: SPInstance,
    mut rng: Xoshiro256PlusPlus,
    sol_listener: &mut impl SolutionListener,
    terminator: &impl Terminator,
    expl_config: &ExplorationConfig,
    cmpr_config: &CompressionConfig,
) -> SPSolution {
    let mut next_rng = || Xoshiro256PlusPlus::seed_from_u64(rng.next_u64());
    let builder = LBFBuilder::new(instance.clone(), next_rng(), LBF_SAMPLE_CONFIG).construct();

    let expl_term = CombinedTerminator::new(
        terminator.clone(),
        TimedTerminator::new_duration(expl_config.time_limit),
    );
    let mut expl_separator = Separator::new(
        builder.instance,
        builder.prob,
        next_rng(),
        expl_config.separator_config,
    );
    let solutions = exploration_phase(
        &instance,
        &mut expl_separator,
        sol_listener,
        &expl_term,
        expl_config,
    );
    let final_explore_sol = solutions.last().unwrap().clone();

    let cmpr_term = CombinedTerminator::new(
        terminator.clone(),
        TimedTerminator::new_duration(cmpr_config.time_limit),
    );
    let mut cmpr_separator = Separator::new(
        expl_separator.instance,
        expl_separator.prob,
        next_rng(),
        cmpr_config.separator_config,
    );
    let cmpr_sol = compression_phase(
        &instance,
        &mut cmpr_separator,
        &final_explore_sol,
        sol_listener,
        &cmpr_term,
        cmpr_config,
    );

    sol_listener.report(ReportType::Final, &cmpr_sol, &instance);

    cmpr_sol
}

#[derive(Debug)]
pub struct OptimizeWorker {
    terminate_flag: Arc<AtomicBool>,
    waiter: Arc<Event>,
    result: Arc<Mutex<Option<SPSolution>>>,
    _thread: std::thread::JoinHandle<()>,
}

impl OptimizeWorker {
    pub fn new(
        instance: SPInstance,
        rng: Xoshiro256PlusPlus,
        sol_listener: impl SolutionListener + Send + Sync + 'static ,
        terminator: impl Terminator + Send + Sync + 'static ,
        expl_config: ExplorationConfig,
        cmpr_config: CompressionConfig,
    ) -> Self {
        let terminate_flag = Arc::new(AtomicBool::new(false));
        let waiter = Arc::new(Event::new());
        let thread_waiter = waiter.clone();
        let result = Arc::new(Mutex::new(None));
        let thread_result = result.clone();

        let terminator = CombinedTerminator::new(
            terminator,
            FlagTerminator::of(terminate_flag.clone()),
        );

        let thread = std::thread::spawn(move || {
            let mut local_listener = sol_listener;
            let local_terminator = terminator;

            let solution = optimize(
                instance,
                rng,
                &mut local_listener,
                &local_terminator,
                &expl_config,
                &cmpr_config,
            );

            thread_waiter.notify(usize::MAX);
            *thread_result.lock().expect("OptimizeWorker mutex was poisoned") = Some(solution);
        });

        OptimizeWorker {
            terminate_flag,
            waiter,
            result,
            _thread: thread,
        }
    }

    fn pull_result(&self) -> Option<SPSolution> {
        self.result
            .lock()
            .expect("OptimizeWorker mutex was poisoned")
            .take()
    }

    pub fn wait(&self) -> Option<SPSolution> {
        if let Some(sol) = self.pull_result() {
            return Some(sol);
        }
        let _ = self.waiter.listen().wait();
        self.result
            .lock()
            .expect("OptimizeWorker mutex was poisoned")
            .take()
    }

    pub fn wait_timeout(&self, duration: Duration) -> Option<SPSolution> {
        if let Some(sol) = self.pull_result() {
            return Some(sol);
        }
        let listener = self.waiter.listen();
        if listener.wait_timeout(duration).is_some() {
            self.pull_result()
        } else {
            None
        }
    }

    pub fn terminate(&self) {
        self.terminate_flag.store(true, Ordering::Relaxed);
    }
}