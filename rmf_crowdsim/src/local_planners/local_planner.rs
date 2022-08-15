use crate::map_representation::map::Map;
use crate::Agent;
use crate::AgentId;
use crate::Vec2f;

use std::sync::{Arc, Mutex};

pub trait LocalPlanner<M: Map> {
    fn get_desired_velocity(
        &self,
        agent: &Agent,
        nearby_agents: &Vec<Agent>,
        recommended_velocity: Vec2f,
        map: Arc<Mutex<M>>,
    ) -> Vec2f;

    fn add_agent(&mut self, _id: AgentId) {}

    fn remove_agent(&mut self, _id: AgentId) {}
}
