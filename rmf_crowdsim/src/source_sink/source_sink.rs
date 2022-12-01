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

use rand::distributions::{Distribution, Uniform};

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

    /// The initial orientation
    pub orientation: f64,

    /// Prefix for naming agents from this source/sink
    pub prefix: String,

    /// The model spawned by this SourceSink
    pub model: String,

    /// The size of the box in which spawning may happen.
    /// Uniform distribution will be used.
    pub source_range: Vec2f,

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

impl SourceSink {
    pub fn spawn(&self) -> (Vec2f, f64) {
        if self.source_range[0] <= 0.0 && self.source_range[1] <= 0.0 {
            return (self.source, self.orientation);
        }

        let mut rng = rand::thread_rng();
        let dx = if self.source_range[0] <= 0.0 {
            0.0
        } else {
            let range_x = self.source_range[0]/2.0;
            Uniform::from(-range_x..range_x).sample(&mut rng)
        };
        let dy = if self.source_range[1] <= 0.0 {
            0.0
        } else {
            let range_y = self.source_range[1]/2.0;
            Uniform::from(-range_y..range_y).sample(&mut rng)
        };

        (self.source + Vec2f::new(dx, dy), self.orientation)
    }
}

/// A crowd generator that uses the poisson function.
pub struct PoissonCrowd {
    pub rate: f64,
}

/// Create a crowd.
impl PoissonCrowd {
    pub fn new(rate: f64) -> Self {
        PoissonCrowd { rate }
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
        MonotonicCrowd { rate }
    }
}

impl CrowdGenerator for MonotonicCrowd {
    fn get_number_to_spawn(&self, time_elapsed: Duration) -> usize {
        let num_spawned = time_elapsed.as_secs_f64() * self.rate;
        num_spawned.round() as usize
    }
}
