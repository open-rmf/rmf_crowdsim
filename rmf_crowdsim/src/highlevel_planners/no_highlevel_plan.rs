use super::highlevel_planners::HighLevelPlanner;
use crate::{Agent, Vec2f};

pub struct NoHighLevelPlan;

impl HighLevelPlanner for NoHighLevelPlan {
    fn get_desired_velocity(&mut self, _: &Agent, _: std::time::Duration) -> Option<Vec2f> {
        Some(Vec2f::zeros())
    }

    fn set_target(&mut self, _: &Agent, _: Vec2f, _: Vec2f) {
        // Do nothing
    }

    fn remove_agent_id(&mut self, _: crate::AgentId) {
        // Do nothing
    }
}
