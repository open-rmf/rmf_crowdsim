use std::sync::{Arc, Mutex};
use std::collections::HashMap;
pub extern crate nalgebra as na;
use na::Vector2;

pub mod local_planners;
pub mod map_representation;
pub mod spatial_index;
pub mod highlevel_planners;

pub use crate::map_representation::map::Map;
pub use crate::local_planners::local_planner::LocalPlanner;
pub use crate::spatial_index::spatial_index::SpatialIndex;
pub use crate::highlevel_planners::highlevel_planners::HighLevelPlanner;

/// Agent  ID
pub type AgentId = usize;

/// Point
/// TODO(arjo) Make generic so we support lifts and multilevels
pub type Point = Vector2<f64>;

/// 2-vector
pub type Vec2f = Vector2<f64>;

/// Data representing an individual agent
#[derive(Clone, Debug)]
pub struct Agent {
    /// Unique Agent ID
    pub agent_id : AgentId,
    /// Position of a point
    pub position : Point,
    /// Orientation of agent
    pub orientation: f64,
    /// Velocity of agent
    pub velocity: Vector2<f64>,
    /// Preferred velocity
    preferred_vel: Vec2f,
    /// Angular velocity of agent
    pub angular_vel: f64,
    /// Eyesight range: How far each individual agent can "see"
    pub eyesight_range: f64
}

/// A representation of a simulation session
pub struct Simulation<M: Map, T: SpatialIndex> {
    /// List of all active agents
    pub agents: Vec<Agent>,
    /// Spatial Index. Used internally to
    spatial_index: T,
    /// Map ARC
    map: Arc<M>,
    /// High level planning
    high_level_planner: HashMap<AgentId, Arc<Mutex<dyn HighLevelPlanner<M>>>>,
    local_planner: HashMap<AgentId, Arc<dyn LocalPlanner<M>>>,
    sim_time: std::time::Duration
}

impl<M: Map, T: SpatialIndex> Simulation<M, T> {

    pub fn new(map: Arc<M>, spatial_index: T) -> Self
    {
        Self {
            agents: vec!(),
            map: map,
            spatial_index: spatial_index,
            high_level_planner: HashMap::new(),
            local_planner: HashMap::new(),
            sim_time: std::time::Duration::new(0,0)
        }
    }

    pub fn add_agents(
        &mut self,
        spawn_positions: &Vec<Point>,
        high_level_planner: Arc<Mutex<dyn HighLevelPlanner<M>>>,
        local_planner: Arc<dyn LocalPlanner<M>>,
        agent_eyesight_range: f64) -> Result<Vec<AgentId>, String>
    {
        let mut res = Vec::<AgentId>::new();
        for x in spawn_positions
        {
            let agent_id = self.agents.len();
            // HUGE RED FLAGS
            high_level_planner.lock().unwrap().set_map(self.map.clone());
            self.high_level_planner.insert(agent_id, high_level_planner.clone());
            self.local_planner.insert(agent_id, local_planner.clone());
            self.agents.push(
                Agent {
                    agent_id: agent_id,
                    position: *x,
                    orientation: 0f64,
                    velocity: Vector2::<f64>::new(0f64, 0f64),
                    preferred_vel: Vector2::<f64>::new(0f64, 0f64),
                    angular_vel: 0f64,
                    eyesight_range: agent_eyesight_range
                }
            );
            let success = self.spatial_index.add_or_update(agent_id, *x);
            if let Err(error_message) = success
            {
                return Err(error_message)
            }
            res.push(agent_id);
        }
        Ok(res)
    }

    pub fn step (&mut self, dur: std::time::Duration) -> Result<(),String>
    {
        for x in 0..self.agents.len()
        {
            let agent_id = self.agents[x].agent_id;

            // Execute the high level plan
            let mut vel = Vec2f::new(0f64, 0f64);
            if let Some(lock) = self.high_level_planner.get(&agent_id) {
                let result  =
                    lock.lock().unwrap().get_desired_velocity(&self.agents[x], self.sim_time);
                if let Some(res) = result
                {
                    vel = res;
                    self.agents[x].preferred_vel = vel.clone();
                }
            }

            // Execute the local planner
            if let Some(local_planner) = self.local_planner.get(&agent_id) {
                let neighbour_ids = self.spatial_index.get_neighbours_in_radius(
                    self.agents[x].eyesight_range,
                    self.agents[x].position);

                let neighbours = Vec::from_iter(neighbour_ids.iter()
                  .filter(|agent_id| **agent_id != x)
                  .map(|agent_id| self.agents[*agent_id].clone()));

                vel = local_planner.get_desired_velocity(
                    &self.agents[x], &neighbours, vel, self.map.clone());
            }

            // Agent position
            let dx = vel * dur.as_secs_f64();
            self.agents[x].position += dx;
            self.agents[x].velocity = vel;

            let success = self.spatial_index.add_or_update(agent_id, self.agents[x].position);
            if let Err(error_message) = success
            {
                return Err(error_message)
            }
        }
        Ok(())
    }
}

