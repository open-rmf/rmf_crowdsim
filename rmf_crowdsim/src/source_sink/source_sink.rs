use crate::Vec2f;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use crate::highlevel_planners::highlevel_planners::HighLevelPlanner;
use crate::local_planners::local_planner::LocalPlanner;

use rand::distributions::Distribution;

use statrs::distribution::Poisson;

/// Trait for crowd generation
pub trait CrowdGenerator {
    /// Gets the number of pedestrians to spawn at a given time.
    fn get_number_to_spawn(&self, time_elapsed: Duration) -> usize;
}

/// Serves as a source and a sink component.
pub struct SourceSink {
    /// The source location
    pub source: Vec2f,

    /// The sink location
    pub sink: Vec2f,

    /// The radius of the sink location
    pub radius_sink: f64,

    /// Generate crowds
    pub crowd_generator: Arc<dyn CrowdGenerator>,

    /// High level planning
    pub high_level_planner: Arc<Mutex<dyn HighLevelPlanner>>,

    /// Local avoidance strategy
    pub local_planner: Arc<Mutex<dyn LocalPlanner>>,

    /// Eyesight (TODO(arjo): Replace with AgentProperties)
    pub agent_eyesight_range: f64
}

pub struct PoissonCrowd {
    pub rate: f64,
}

impl PoissonCrowd {
    pub fn new(rate: f64) -> Self {
        PoissonCrowd { rate: rate }
    }
}

impl CrowdGenerator for PoissonCrowd {
    fn get_number_to_spawn(&self, time_elapsed: Duration) -> usize {
        let rt = time_elapsed.as_secs_f64() * self.rate;
        //println!("Poisson rate {}", rt);
        let mut rng = rand::thread_rng();
        let n = Poisson::new(rt).unwrap();
        n.sample(&mut rng) as usize
    }
}

pub struct MonotonicCrowd {
    pub rate: f64,
}

impl MonotonicCrowd {
    pub fn new(rate: f64) -> Self {
        MonotonicCrowd { rate: rate }
    }
}

impl CrowdGenerator for MonotonicCrowd {
    fn get_number_to_spawn(&self, time_elapsed: Duration) -> usize {
        let num_spawned = time_elapsed.as_secs_f64() * self.rate;
        num_spawned.round() as usize
    }
}
