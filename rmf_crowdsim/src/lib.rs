use std::collections::HashMap;
use std::sync::{Arc, Mutex};
pub extern crate nalgebra as na;
use na::Vector2;

pub mod highlevel_planners;
pub mod local_planners;
pub mod map_representation;
pub mod spatial_index;

pub use crate::highlevel_planners::highlevel_planners::HighLevelPlanner;
pub use crate::local_planners::local_planner::LocalPlanner;
pub use crate::map_representation::map::Map;
pub use crate::spatial_index::spatial_index::SpatialIndex;

/// Agent  ID
pub type AgentId = usize;

/// Point
/// TODO(arjo) Make generic so we support lifts and multilevels
pub type Point = Vector2<f64>;

/// 2-vector
pub type Vec2f = Vector2<f64>;

/// Data representing an individual agent
#[derive(Clone, Copy, Debug)]
pub struct Agent {
    /// Unique Agent ID
    pub agent_id: AgentId,
    /// Position of a point
    pub position: Point,
    /// Orientation of agent
    pub orientation: f64,
    /// Velocity of agent
    pub velocity: Vector2<f64>,
    /// Preferred velocity
    preferred_vel: Vec2f,
    /// Angular velocity of agent
    pub angular_vel: f64,
    /// Eyesight range: How far each individual agent can "see"
    pub eyesight_range: f64,
}

/// A representation of a simulation session
pub struct Simulation<M: Map, T: SpatialIndex> {
    /// List of all active agents
    pub agents: HashMap<AgentId, Agent>,
    /// Spatial Index. Used internally to
    spatial_index: T,
    /// Map ARC
    map: Arc<M>,
    /// High level planning
    high_level_planner: HashMap<AgentId, Arc<Mutex<dyn HighLevelPlanner<M>>>>,
    /// Local avoidance strategy
    local_planner: HashMap<AgentId, Arc<Mutex<dyn LocalPlanner<M>>>>,
    /// Simulation time
    sim_time: std::time::Duration,
    /// Get last allocated agent id
    last_alloc_agent_id: usize,
    /// Update buffer: Ideally we would not need this if I had done a better job of the update loop
    update_buffer: HashMap<AgentId, StateUpdateBuffer>,
}

/// Just to keep rust happy TODO: Remove this
struct StateUpdateBuffer {
    new_vel: Vec2f,
    new_pos: Vec2f,
    updated: bool,
}

///
impl<M: Map, T: SpatialIndex> Simulation<M, T> {
    /// Create a new simulation environment
    pub fn new(map: Arc<M>, spatial_index: T) -> Self {
        Self {
            agents: HashMap::new(),
            map: map,
            spatial_index: spatial_index,
            high_level_planner: HashMap::new(),
            local_planner: HashMap::new(),
            sim_time: std::time::Duration::new(0, 0),
            last_alloc_agent_id: 0,
            update_buffer: HashMap::new(),
        }
    }

    pub fn add_agents(
        &mut self,
        spawn_positions: &Vec<Point>,
        high_level_planner: Arc<Mutex<dyn HighLevelPlanner<M>>>,
        local_planner: Arc<Mutex<dyn LocalPlanner<M>>>,
        agent_eyesight_range: f64,
    ) -> Result<Vec<AgentId>, String> {
        let mut res = Vec::<AgentId>::new();
        for x in spawn_positions {
            let agent_id = self.last_alloc_agent_id;
            self.last_alloc_agent_id += 1;
            // HUGE RED FLAGS
            high_level_planner.lock().unwrap().set_map(self.map.clone());
            self.high_level_planner
                .insert(agent_id, high_level_planner.clone());
            self.local_planner.insert(agent_id, local_planner.clone());
            self.agents.insert(
                agent_id,
                Agent {
                    agent_id: agent_id,
                    position: *x,
                    orientation: 0f64,
                    velocity: Vector2::<f64>::new(0f64, 0f64),
                    preferred_vel: Vector2::<f64>::new(0f64, 0f64),
                    angular_vel: 0f64,
                    eyesight_range: agent_eyesight_range,
                },
            );
            let success = self.spatial_index.add_or_update(agent_id, *x);
            if let Err(error_message) = success {
                return Err(error_message);
            }
            res.push(agent_id);
        }
        Ok(res)
    }

    pub fn remove_agents(&mut self, agent: AgentId) {
        self.high_level_planner[&agent]
            .lock()
            .unwrap()
            .remove_agent_id(agent);
        self.local_planner[&agent]
            .lock()
            .unwrap()
            .remove_agent(agent);
        self.agents.remove_entry(&agent);
        self.update_buffer.remove_entry(&agent);
    }

    pub fn step(&mut self, dur: std::time::Duration) -> Result<(), String> {
        for agent_id in self.agents.keys() {
            let mut agent = self.agents.get(agent_id).unwrap().clone();
            // Execute the high level plan
            let mut vel = Vec2f::new(0f64, 0f64);
            if let Some(lock) = self.high_level_planner.get(&agent_id) {
                let result = lock
                    .lock()
                    .unwrap()
                    .get_desired_velocity(&agent, self.sim_time);
                if let Some(res) = result {
                    vel = res;
                    agent.preferred_vel = vel.clone();
                }
            }

            // Execute the local planner
            if let Some(local_planner) = self.local_planner.get(&agent_id) {
                let neighbour_ids = self
                    .spatial_index
                    .get_neighbours_in_radius(agent.eyesight_range, agent.position);

                let neighbours = Vec::from_iter(
                    neighbour_ids
                        .iter()
                        .filter(|neighbor_id| **neighbor_id != *agent_id)
                        .map(|neighbor_id| self.agents.get(neighbor_id).unwrap().clone()),
                );

                vel = local_planner.lock().unwrap().get_desired_velocity(
                    &agent,
                    &neighbours,
                    vel,
                    self.map.clone(),
                );
            }

            // Agent position
            let dx = vel * dur.as_secs_f64();
            let pos = agent.position;
            let new_pos = pos + dx;

            // let mut agent_mut = self.agents.get_mut(agent_id).unwrap();
            // agent_mut.position = new_pos;
            // agent_mut.velocity = vel;
            self.update_buffer.insert(
                *agent_id,
                StateUpdateBuffer {
                    new_pos: new_pos,
                    new_vel: vel,
                    updated: true,
                },
            );

            let success = self.spatial_index.add_or_update(*agent_id, new_pos);
            if let Err(error_message) = success {
                return Err(error_message);
            }
        }

        for (id, state_update) in self.update_buffer.iter_mut() {
            if !state_update.updated {
                continue;
            }
            let mut agent = self.agents.get_mut(id).unwrap();
            agent.velocity = state_update.new_vel;
            agent.position = state_update.new_pos;
            state_update.updated = false;
        }

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    // Note this useful idiom: importing names from outer (for mod tests) scope.
    use super::*;

    struct NoMap {}

    impl Map for NoMap {
        fn get_occupancy(&self, _pt: Point) -> Option<bool> {
            return Some(true);
        }
    }

    struct StubHighLevelPlan {
        default_vel: Vec2f,
    }

    impl StubHighLevelPlan {
        fn new(default_vel: Vec2f) -> Self {
            StubHighLevelPlan {
                default_vel: default_vel,
            }
        }
    }

    impl<M: Map> HighLevelPlanner<M> for StubHighLevelPlan {
        fn get_desired_velocity(
            &mut self,
            _agent: &Agent,
            _time: std::time::Duration,
        ) -> Option<Vec2f> {
            Some(self.default_vel)
        }

        /// Set the target position for a given agent
        fn set_target(&mut self, _agent: &Agent, _point: Point, _tolerance: Vec2f) {
            // For now do nothing
        }
        /// Remove an agent
        fn remove_agent_id(&mut self, _agent: AgentId) {
            // Do nothing
        }

        fn set_map(&mut self, _map: Arc<M>) {
            // Do nothing
        }
    }

    #[test]
    fn test_step_integration() {
        let map = Arc::new(NoMap {});
        let velocity = Vec2f::new(1.0f64, 0.0f64);
        let step_size = std::time::Duration::new(1, 0);
        let stub_spatial = spatial_index::location_hash_2d::LocationHash2D::new(
            1000f64,
            1000f64,
            20f64,
            Point::new(-500f64, -500f64),
        );
        let mut crowd_simulation = Simulation::new(map, stub_spatial);
        let agent_start_positions = vec![Point::new(0f64, 0f64)];
        let high_level_planner = Arc::new(Mutex::new(StubHighLevelPlan::new(velocity)));
        let mut local_planner = Arc::new(Mutex::new(local_planners::no_local_plan::NoLocalPlan {}));

        assert_eq!(crowd_simulation.agents.len(), 0u64 as usize);

        let agents = crowd_simulation.add_agents(
            &agent_start_positions,
            high_level_planner,
            local_planner,
            100f64,
        );
        assert_eq!(crowd_simulation.agents.len(), 1u64 as usize);

        let res = crowd_simulation.step(step_size);
        assert_eq!(res, Ok(()));
        assert_eq!(crowd_simulation.agents.len(), 1u64 as usize);

        assert!((crowd_simulation.agents[&(0u64 as usize)].position - velocity).norm() < 1e-5f64);
    }
}
