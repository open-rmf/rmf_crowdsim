use crate::Agent;
use crate::Vec2f;
use crate::map_representation::map::Map;
use crate::spatial_index::spatial_index::SpatialIndex;

use std::sync::Arc;

pub trait LocalPlanner<M : Map, T: SpatialIndex> {
    fn get_desired_velocity(&self,
        agent: &Agent,
        recommended_velocity: Vec2f,
        spatial_index: &T,
        map: Arc<M>) -> Vec2f;
}
