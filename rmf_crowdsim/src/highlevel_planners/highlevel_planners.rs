use crate::map_representation::map::Map;
use crate::Agent;
use crate::AgentId;
use crate::Point;
use crate::Vec2f;

use std::sync::{Arc, Mutex};

pub trait HighLevelPlanner<M: Map> {
    fn get_desired_velocity(&mut self, agent: &Agent, time: std::time::Duration) -> Option<Vec2f>;

    /// Set the target position for a given agent
    fn set_target(&mut self, agent: &Agent, point: Point, tolerance: Vec2f);

    /// Remove an agent
    fn remove_agent_id(&mut self, _agent: AgentId) {}

    fn set_map(&mut self, map: Arc<Mutex<M>>);
}
