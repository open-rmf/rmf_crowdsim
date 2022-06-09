use std::sync::{Arc, Mutex};
use std::collections::HashMap;
pub extern crate nalgebra as na;
use na::Vector2;

pub mod local_planners;
pub mod map_representation;

pub use crate::map_representation::map::Map;
pub use crate::local_planners::local_planner::LocalPlanner;

/// Agent  ID
pub type AgentId = usize;

/// Point
pub type Point = Vector2<f64>;

/// 2-vector
pub type Vec2f = Vector2<f64>;

/// Data representing an individual agent
pub struct Agent {
    agent_id : AgentId,
    pub position : Point,
    pub orientation: f64,
    pub velocity: Vector2<f64>,
    pub angular_vel: f64
}


/// Abstract interface for planners
pub trait HighLevelPlanner<M :Map> {

    fn get_desired_velocity(&mut self, agent: &Agent, time: std::time::Duration) -> Option<Vec2f>;

    /// Set the target position for a given agent
    fn set_target(&mut self, agent: &Agent, point: Point, tolerance: Vec2f);

    /// Remove an agent
    fn remove_agent_id(&mut self, agent: AgentId);


    fn set_map(&mut self, map: Arc<M>);
}


pub struct Simulation<M: Map> {
    pub agents: Vec<Agent>,
    map: Arc<M>,
    high_level_planner: HashMap<AgentId, Arc<Mutex<dyn HighLevelPlanner<M>>>>,
    local_planner: HashMap<AgentId, Arc<dyn LocalPlanner<M>>>,
    sim_time: std::time::Duration
}

impl<M: Map> Simulation<M> {

    pub fn new(map: Arc<M>) -> Self
    {
        Self {
            agents: vec!(),
            map: map,
            high_level_planner: HashMap::new(),
            local_planner: HashMap::new(),
            sim_time: std::time::Duration::new(0,0)
        }
    }

    pub fn add_agents(
        &mut self,
        spawn_positions: &Vec<Point>,
        high_level_planner: Arc<Mutex<dyn HighLevelPlanner<M>>>,
        local_planner: Arc<dyn LocalPlanner<M>>) -> Vec<AgentId>
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
                    angular_vel: 0f64
                }
            );
            res.push(agent_id);
        }
        res
    }

    pub fn step (&mut self, dur: std::time::Duration)
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
                }
            }

            // Execute the local
            if let Some(local_planner) = self.local_planner.get(&agent_id) {
                vel = local_planner.get_desired_velocity(&self.agents[x], vel, self.map.clone());
            }

            // Agent position
            let dx = vel * dur.as_secs_f64();
            self.agents[x].position += dx;
            self.agents[x].velocity = vel;
        }
    }
}

