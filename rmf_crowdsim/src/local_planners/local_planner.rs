use crate::map_representation::map::Map as Map;
use crate::Agent;
use crate::Vec2f;
use std::sync::Arc;

pub trait LocalPlanner<M: Map> {
    fn get_desired_velocity(
        &self, agent: &Agent, recommended_velocity: Vec2f, map: Arc<M>) -> Vec2f;
}

