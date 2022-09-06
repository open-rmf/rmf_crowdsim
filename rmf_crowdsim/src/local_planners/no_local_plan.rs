use crate::local_planners::local_planner::LocalPlanner;
use crate::Agent;
use crate::Vec2f;

use std::sync::{Arc, Mutex};

pub struct NoLocalPlan {}

impl LocalPlanner for NoLocalPlan {
    fn get_desired_velocity(
        &self,
        _agent: &Agent,
        _nearby_agents: &Vec<Agent>,
        recommended_velocity: Vec2f,
    ) -> Vec2f {
        recommended_velocity
    }
}
