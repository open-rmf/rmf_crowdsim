use yaml_rust::YamlLoader;

use image::{GrayImage, Luma, Rgb, RgbImage};
use line_drawing::Bresenham;

use std::collections::{HashMap, HashSet, VecDeque};
use std::sync::Arc;

use nalgebra::Vector2;

use mapf::{
    algorithm::AStarConnect,
    graph::{
        occupancy::{Cell, Grid, Point, SparseGrid, Visibility, VisibilityGraph},
        SharedGraph,
    },
    algorithm::SearchStatus,
    motion::{
        Trajectory,
        r2::{LineFollow, WaypointR2},
    },
    premade::SearchR2,
    Planner,
};

use crate::Agent;
use crate::AgentId;
use crate::Vec2f;

use crate::highlevel_planners::highlevel_planners::HighLevelPlanner;

////////////////////////////////////////////////////////////////////////////////
/// SpatialHash used for caching later on
#[derive(Debug, Clone, Copy, Hash, PartialEq, Eq)]
struct SpatialHash {
    x: i64,
    y: i64,
}

impl SpatialHash {
    fn new(x: f64, y: f64, res: f64) -> Self {
        Self {
            x: (x / res).round() as i64,
            y: (y / res).round() as i64,
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
/// High level planner for an RMF-world.
pub struct RMFPlanner {
    /// The visibility graph
    visibility_graph: Arc<Visibility<SparseGrid>>,
    /// Routes as saved by Agents. (route_index, next_waypoint)
    agent_cache: HashMap<AgentId, (usize, usize)>,
    /// Routes
    route_list: Vec<Vec<Vec2f>>,
    /// By source and target.
    route_plans_by_location: HashMap<(SpatialHash, SpatialHash), usize>,
    /// Scale of spatial hash
    scale: f64,
    /// Agent radius
    radius: f64,
}

impl RMFPlanner {
    /// Create a new planner using
    pub fn new(
        vertices: Vec<(f64, f64)>,
        walls: Vec<(usize, usize)>,
        scale: f64,
        radius: f64,
    ) -> Self {
        let mut grid = SparseGrid::new(scale);

        for (v1, v2) in walls {
            let start = vertices[v1];
            let end = vertices[v2];
            let start = (
                (start.0 / scale).floor() as i64,
                (start.1 / scale).floor() as i64,
            );
            let end = (
                (end.0 / scale).floor() as i64,
                (end.1 / scale).floor() as i64,
            );
            let mut cells_to_update = HashMap::default();
            for (x, y) in Bresenham::new(start, end) {
                cells_to_update.insert(Cell::new(x, y), true);
            }
            grid.change_cells(&cells_to_update);
        }

        Self {
            visibility_graph: Arc::new(Visibility::<SparseGrid>::new(grid, radius)),
            agent_cache: HashMap::default(),
            route_list: vec![],
            route_plans_by_location: HashMap::default(),
            scale,
            radius,
        }
    }

    /// Read yaml format.
    /// TODO(arjo): make optional
    pub fn from_yaml(yaml_str: &str, inflation: f64, scale: f64, agent_radius: f64) -> Self {
        let mut vertices = vec![];

        let doc = YamlLoader::load_from_str(yaml_str).unwrap();

        for v in doc[0]["levels"]["L1"]["vertices"].as_vec().unwrap() {
            let x = v[0].as_f64().unwrap();
            let y = v[1].as_f64().unwrap();
            vertices.push((x, y));
        }

        let mut walls = vec![];
        for wall in doc[0]["levels"]["L1"]["walls"].as_vec().unwrap() {
            let wall = (
                wall[0].as_i64().unwrap() as usize,
                wall[1].as_i64().unwrap() as usize,
            );
            walls.push(wall);
        }

        Self::new(vertices, walls, scale, agent_radius)
    }

    fn plan_route(&self, start: (f64, f64), end: (f64, f64)) -> Option<Vec<Vec2f>> {
        let start_pos = Point::new(start.0, start.1);
        let end_pos = Point::new(end.0, end.1);

        let start_cell = Cell::from_point(start_pos, self.scale);
        let end_cell = Cell::from_point(end_pos, self.scale);

        let graph = SharedGraph::new(VisibilityGraph::new(
            self.visibility_graph.clone(),
            [start_cell, end_cell].into_iter(),
        ));

        let planner = Planner::new(AStarConnect(
            SearchR2::new_r2(graph, LineFollow::new(1.0f64).unwrap())
        ));
        let search = planner.plan(start_cell, end_cell);

        search.ok().map(|mut s| s.solve().ok().map(|s| s.solution()).flatten()).flatten()
        .map(|solution| {
            let trajecory: Option<Trajectory<WaypointR2>> = solution.make_trajectory().ok()
                .map(|t| t.map(|t| t.trajectory))
                .flatten();

            let result: Option<Vec<Vec2f>> = trajecory.map(|t| {
                t.iter().map(|wp| wp.position.coords).collect()
            });

            result
        })
        .flatten()
    }
}

impl HighLevelPlanner for RMFPlanner {
    /// Get the desired
    fn get_desired_velocity(&mut self, agent: &Agent, time: std::time::Duration) -> Option<Vec2f> {
        if let Some(path) = self.agent_cache.get(&agent.agent_id) {
            let path_id = path.0;
            let mut waypoint_id = path.1;
            // TODO(arjo):
            if (agent.position - self.route_list[path_id][waypoint_id]).norm() < 1e-1
                && self.route_list[path_id].len() > waypoint_id + 1
            {
                waypoint_id += 1;
                self.agent_cache
                    .insert(agent.agent_id, (path_id, waypoint_id));
            }
            return Some((self.route_list[path_id][waypoint_id] - agent.position).normalize());
        } else {
            // TODO(arjo): Replan route if agent is outside route.
            //println!("Agent {:?} was not found", agent.agent_id);
            return None;
        }
    }
    /// Set the target position for a given agent
    fn set_target(&mut self, agent: &Agent, point: Vec2f, tolerance: Vec2f) {
        let start_pos = agent.position;
        let start_hash = SpatialHash::new(start_pos.x, start_pos.y, self.scale);

        let end_hash = SpatialHash::new(point.x, point.y, self.scale);
        if let Some(idx) = self.route_plans_by_location.get(&(start_hash, end_hash)) {
            self.agent_cache.insert(agent.agent_id, (*idx, 0));
        } else {
            let result = self.plan_route((start_pos.x, start_pos.y), (point.x, point.y));
            if let Some(result) = result {
                self.route_plans_by_location
                    .insert((start_hash, end_hash), self.route_list.len());
                self.agent_cache
                    .insert(agent.agent_id, (self.route_list.len(), 0));
                println!("{:?}", result);
                self.route_list.push(result);
            } else {
                println!("Failed to find contiguous path between source and target");
            }
        }
    }
    /// Remove an agent
    fn remove_agent_id(&mut self, agent: AgentId) {
        self.agent_cache.remove(&agent);
    }
}
