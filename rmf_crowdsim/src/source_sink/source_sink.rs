/*
 * Copyright (C) 2022 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

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

    /// The radius of the sink location
    pub radius_sink: f64,

    /// Generate crowds
    pub crowd_generator: Arc<dyn CrowdGenerator>,

    /// High level planning
    pub high_level_planner: Arc<Mutex<dyn HighLevelPlanner>>,

    /// Local avoidance strategy
    pub local_planner: Arc<Mutex<dyn LocalPlanner>>,

    /// Waypoints. The last waypoint acts as a sink.
    pub waypoints: Vec<Vec2f>,

    /// Loop through waypoints if have value.
    pub loop_forever: bool,

    /// Eyesight (TODO(arjo): Replace with AgentProperties)
    pub agent_eyesight_range: f64,
}

/// A crowd generator that uses the poisson function.
pub struct PoissonCrowd {
    pub rate: f64,
}

/// Create a crowd.
impl PoissonCrowd {
    pub fn new(rate: f64) -> Self {
        PoissonCrowd { rate: rate }
    }
}

/// Get number to spawn in a given dt.
impl CrowdGenerator for PoissonCrowd {
    fn get_number_to_spawn(&self, time_elapsed: Duration) -> usize {
        let rt = time_elapsed.as_secs_f64() * self.rate;
        let mut rng = rand::thread_rng();
        let n = Poisson::new(rt).unwrap();
        n.sample(&mut rng) as usize
    }
}

/// Monotonic Crowd. Creates a fixed number of people every time step.
pub struct MonotonicCrowd {
    pub rate: f64,
}

impl MonotonicCrowd {
    /// rate: rate at which to spawn.
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
