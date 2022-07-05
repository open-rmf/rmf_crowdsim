use crate::local_planners::local_planner::LocalPlanner;
use crate::map_representation::map::Map;
use crate::spatial_index::spatial_index::SpatialIndex;
use crate::Vec2f;
use crate::Agent;

use std::sync::Arc;

pub struct NoLocalPlan{}

impl<M: Map> LocalPlanner<M> for NoLocalPlan {
    fn get_desired_velocity(&self,
        _agent: &Agent, _nearby_agents: &Vec<Agent>, recommended_velocity: Vec2f, _map: Arc<M>) -> Vec2f
    {
        recommended_velocity
    }
}
