use image::{RgbImage, Rgb, GrayImage, Luma};
use line_drawing::Bresenham;

use nalgebra::{Vector2};

use mapf::{directed::simple::SimpleGraph,  a_star, algorithm::Status,
    planner::make_planner,
    motion::{
        r2::{Position, timed_position::LineFollow, graph_search::make_default_expander}
    }};

pub struct RMFObstacles
{
    location: (f64, f64),
    radius: (f64, f64)
}

pub struct RMFMapFactory {
    vertices: Vec<(f64, f64)>,
    walls: Vec<(usize, usize)>,
    scale: f64
    rmf_obstacles: Vec<RMFObstacles>
}

impl RMFMapFactory {
    /// Create a simulation map from a graph
    /// # Arguments
    /// * vertices: Location of vertices
    /// * wall: between vertex
    /// * scale: The scale of the occupancy grid used for high level planning.
    fn from_graph(
        vertices: &Vec<(f64, f64)>,
        walls: &Vec<(usize, usize)>,
        scale: &f64
    ) ->self
    {
        Self {
            vertices,
            walls,
            scale,
            rmf_obstacles: vec!()
        }
    }
}

struct RMFPlanner {

}