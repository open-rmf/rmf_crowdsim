use crate::Point;
use crate::AgentId;

pub trait SpatialIndex
{
    fn add_or_update(&mut self, index: AgentId, position: Point)-> Result<(),String>;

    fn get_nearest_neighbours(&self, n: usize, position: Point)-> Vec<AgentId>;

    fn get_neighbours_in_radius(&self, radius: f64, position: Point) -> Vec<AgentId>;

    fn remove_agent(&mut self, _agent: AgentId) {
        // Do Nothing
    }
}
