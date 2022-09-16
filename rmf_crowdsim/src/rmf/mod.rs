use yaml_rust::{YamlLoader};

use image::{RgbImage, Rgb, GrayImage, Luma};
use line_drawing::Bresenham;

use std::collections::{HashMap, HashSet, VecDeque};
use std::sync::Arc;

use imageproc::{contours::{find_contours}, point::Point};

use nalgebra::{Vector2};

use mapf::{directed::simple::SimpleGraph,  a_star, algorithm::Status,
    planner::make_planner,
    motion::{
        r2::{Position, timed_position::LineFollow, graph_search::make_default_expander}
    }};

use crate::Agent;
use crate::AgentId;
use crate::Vec2f;

use crate::highlevel_planners::highlevel_planners::HighLevelPlanner;

#[derive(Debug, Clone, Copy, Hash, PartialEq, Eq)]
struct SpatialHash{
    x: i64,
    y: i64
}

impl SpatialHash
{
    fn new(x: f64, y: f64, res: f64) -> Self
    {
        Self {
            x: (x / res).round() as i64,
            y: (y / res).round() as i64
        }
    }
}

struct VisibilityGraph {
    obstacle_map: ObstacleMap,
    color_cache: RgbImage,
    edges: Vec<(usize, usize)>,
    scale: f64
}

impl VisibilityGraph {

    fn as_image(&self) -> RgbImage {
        let mut img = self.obstacle_map.as_image();
        let lower_bounds = self.obstacle_map.get_lower_bounds();

        for (e1, e2) in &self.edges {
            let coord1 = self.obstacle_map.to_image_coordinates(
                self.obstacle_map.vertices[*e1], lower_bounds);
            let coord2 = self.obstacle_map.to_image_coordinates(
                self.obstacle_map.vertices[*e2], lower_bounds);

            let start = (coord1.0 as i32, coord1.1 as i32);
            let end = (coord2.0 as i32, coord2.1 as i32);
            for (x, y) in Bresenham::new(start, end) {
                img.put_pixel(x as u32, y as u32, Rgb([0, 0, 255]));
            }
        }
        img
    }

    fn plan_route(&self, start: (f64, f64), end: (f64, f64)) -> Option<Vec<Vec2f>> {
        let start = (start.0 * self.scale, start.1 * self.scale);
        let end = (end.0 * self.scale, end.1 * self.scale);
        let start_time =  std::time::SystemTime::now();
        let mut visibility = vec!();
        let mut new_verts = self.obstacle_map.vertices.clone();
        let lower_bounds = self.obstacle_map.get_lower_bounds();
        new_verts.push(start);
        new_verts.push(end);
        for v1 in self.obstacle_map.vertices.len()..self.obstacle_map.vertices.len()+2 {
            for v2 in 0..v1
            {
                let (start_x, start_y) = self.obstacle_map.to_image_coordinates(new_verts[v1], lower_bounds);
                let (end_x, end_y) = self.obstacle_map.to_image_coordinates(new_verts[v2], lower_bounds);
                let (start_x, start_y) = (start_x as i32, start_y as i32);
                let (end_x, end_y) = (end_x as i32, end_y as i32);
                let start = (start_x, start_y);
                let end = (end_x, end_y);
                let mut visible = true;
                for (x, y) in Bresenham::new(start, end) {

                    if (x - start_x) * (x - start_x) + (y - start_y) * (y - start_y) < 4 {
                        continue;
                    }

                    if (x - end_x) * (x - end_x) + (y - end_y) * (y - end_y) < 4 {
                        continue;
                    }

                    if x < 0 || y < 0 || x > self.color_cache.width() as i32 || y > self.color_cache.height() as i32
                    {
                        continue;
                    }

                    if *self.color_cache.get_pixel(x as u32, y as u32) != Rgb([0u8,0u8,255u8])
                    {
                        visible = false;
                    }
                }

                if visible {
                    visibility.push((v1,v2));
                    visibility.push((v2,v1));
                }
            }
        }

        let num_verts = new_verts.len();

        let mut mapf_verts = vec!();
        for v in new_verts {
            mapf_verts.push(Position::new(v.0, v.1));
        }

        let mut edges = Vec::<Vec::<usize>>::new();

        // Add new edges
        for v in visibility {
            edges.push(vec!(v.0, v.1));
        }

        // And old edges
        for v in &self.edges {
            edges.push(vec!(v.0, v.1));
        }

        let graph = SimpleGraph::new(mapf_verts, edges);
        let expander = make_default_expander(Arc::new(graph), Arc::new(LineFollow::new(1.0f64).unwrap()));
        let planner = make_planner(Arc::new(expander), Arc::new(a_star::Algorithm));
        let mut progress = planner.plan(&(num_verts-2), num_verts-1).unwrap();
        match progress.solve().unwrap() {
            Status::Solved(solution) => {
                let mut waypoints = vec!();
                let motion = solution.motion().as_ref().unwrap();
                for waypoint in motion.iter() {
                    waypoints.push(Vec2f::new(waypoint.position.x, waypoint.position.y)/ self.scale)
                }
                return Some(waypoints);
            },
            Status::Impossible => {
                return None;
            },
            Status::Incomplete => {
                return None;
            }
        }
    }
}

#[derive(Clone)]
struct ObstacleMap
{
    vertices: Vec<(f64, f64)>,
    walls: Vec<(usize, usize)>,
    scale: f64
}

impl ObstacleMap {

    fn new() -> Self
    {
        Self{
            vertices: vec!(),
            walls: vec!(),
            scale: 1.0
        }
    }

    fn load_map(yaml_str: &str, scale: f64) -> Self
    {
        let doc = YamlLoader::load_from_str(yaml_str).unwrap();

        let mut vertices = vec!();

        for v in doc[0]["levels"]["L1"]["vertices"].as_vec().unwrap() {
            let x = v[0].as_f64().unwrap() * scale;// TODO(arjo): Remove magic number
            let y = v[1].as_f64().unwrap() * scale;// TODO(arjo): Remove magic number
            vertices.push((x,y));
        }

        let mut walls = vec!();
        for wall in doc[0]["levels"]["L1"]["walls"].as_vec().unwrap() {
            let (wall_v1, wall_v2) = (wall[0].as_i64().unwrap() as usize, wall[1].as_i64().unwrap() as usize);
            walls.push((wall_v1, wall_v2));
        }

        Self {
            vertices,
            walls,
            scale
        }
    }

    fn get_lower_bounds(&self) -> (f64, f64)
    {
        let (mut max_x, mut max_y) = (0f64, 0f64);
        for (x, y) in &self.vertices {
            max_x = x.min(max_x);
            max_y = y.min(max_y);
        }
        (max_x, max_y)
    }

    fn get_upper_bounds(&self) -> (f64, f64)
    {
        let (mut max_x, mut max_y) = (0f64, 0f64);
        for (x, y) in &self.vertices {
            max_x = x.max(max_x);
            max_y = y.max(max_y);
        }
        (max_x, max_y)
    }

    fn vert_wall_correspondence(&self) -> HashMap<usize, Vec<usize>>
    {
        let mut hashmap = HashMap::<usize, Vec<usize>>::new();

        for (v1, v2) in &self.walls
        {
            if let Some(mut vec) = hashmap.get_mut(&v1) {
                vec.push(*v2);
            }
            else {
                hashmap.insert(*v1, vec!(*v2));
            }

            if let Some(mut vec) = hashmap.get_mut(&v2) {
                vec.push(*v1);
            }
            else {
                hashmap.insert(*v2, vec!(*v1));
            }
        }

        hashmap
    }

    fn graph_color(&self) -> (RgbImage, HashSet<usize>)
    {
        let mut grid = self.as_image();
        let mut queue = VecDeque::new();
        let mut visited = HashSet::new();
        let mut wall_pixels = HashSet::new();

        let lower_bounds = self.get_lower_bounds();
        let upper_bounds = self.get_upper_bounds();

        queue.push_back((0u32,0u32));
        while !queue.is_empty()
        {

            if let Some(pt) = queue.pop_front()
            {
                visited.insert(pt);
 
                if *grid.get_pixel(pt.0, pt.1) != Rgb([0u8,0u8,0u8]) {
                    if *grid.get_pixel(pt.0, pt.1) == Rgb([255u8,0u8,0u8]) {
                        wall_pixels.insert((pt.0, pt.1));
                    }
                    continue;
                }

                if pt.0 > 0 {
                    queue.push_back((pt.0 -1, pt.1));
                }
                if pt.1 > 0 {
                    queue.push_back((pt.0 , pt.1 -1));
                }

                if pt.0 < grid.width() - 1 {
                    queue.push_back((pt.0 + 1, pt.1));
                }

                if pt.1 < grid.height() - 1 {
                    queue.push_back((pt.0 , pt.1  + 1));
                }

                grid.put_pixel(pt.0, pt.1, Rgb([0, 0, 255u8]));
            }
        }
        let walls = self.wall_id_by_location(); // Returns <(pixel_position), id>

        //TODO(arjo): Change to int
        let mut visible_wall_id_length = HashMap::<usize, HashSet<(u32, u32)>>::new();
        for wall in wall_pixels {
            let wall_id = walls[&wall];
            let wall_length = visible_wall_id_length.get_mut(&wall_id);
            if let Some(wall_length) = wall_length {
                wall_length.insert(wall);
            }
            else {
                let mut hashset = HashSet::new();
                hashset.insert(wall);
                visible_wall_id_length.insert(wall_id, hashset);
            }
        }

        // weak condition: use result of bfs to mark walls that are visible. A
        // stronger condition would be to traverse the visibility graph and add
        // walls
        let mut wall_ids = HashSet::new();
        for (id, pts) in visible_wall_id_length.iter() {
            let (v1, v2) = self.walls[*id];
            let (x1, y1) = self.to_image_coordinates(
                self.vertices[v1], lower_bounds);
            let (x2, y2) = self.to_image_coordinates(
                self.vertices[v2], lower_bounds);
            // Get supposed length of wall
            let supposed_pixel_length = (x2 as f64 - x1 as f64) * (x2 as f64 - x1 as f64)
                + (y2 as f64 - y1 as f64) * (y2 as f64 - y1 as f64);
            let ratio = pts.len() as f64 / supposed_pixel_length.sqrt();
            if ratio > 0.4 {
                wall_ids.insert(*id);
            }
        }
        (grid, wall_ids)
    }

    fn get_desired_image_dimensions(&self) -> (u32, u32)
    {
        let lower_bounds = self.get_lower_bounds();
        let upper_bounds = self.get_upper_bounds();

        ((upper_bounds.0 - lower_bounds.0) as u32, (upper_bounds.1 - lower_bounds.1) as u32)
    }

    pub fn to_image_coordinates(&self, pos: (f64, f64), lower_bound: (f64, f64)) -> (u32, u32)
    {
        ((pos.0 - lower_bound.0).round() as u32, (pos.1 - lower_bound.1).round() as u32)
    }

    fn visibility_graph(&self) -> VisibilityGraph
    {
        let mut image = self.as_image();
        let (area_of_play, visible_walls) = self.graph_color();
        let lower_bound = self.get_lower_bounds();
        let mut visibility = vec!();

        let last_index = if self.vertices.len() != 0 { self.vertices.len() - 1} else {0};

        for v1 in 0..last_index {
            for v2 in v1+1..self.vertices.len()
            {
                let (start_x, start_y) = self.to_image_coordinates(self.vertices[v1], lower_bound);
                let (end_x, end_y) = self.to_image_coordinates(self.vertices[v2], lower_bound);
                let (start_x, start_y) = (start_x as i32, start_y as i32);
                let (end_x, end_y) = (end_x as i32, end_y as i32);
                let start = (start_x, start_y);
                let end = (end_x, end_y);
                let mut visible = true;
                for (x, y) in Bresenham::new(start, end) {

                    if (x - start_x) * (x - start_x) + (y - start_y) * (y - start_y) < 4 {
                        continue;
                    }

                    if (x - end_x) * (x - end_x) + (y - end_y) * (y - end_y) < 4 {
                        continue;
                    }

                    if x < 0 || y < 0 || x > area_of_play.width() as i32 || y > area_of_play.height() as i32
                    {
                        continue;
                    }

                    if *area_of_play.get_pixel(x as u32, y as u32) != Rgb([0u8,0u8,255u8])
                    {
                        visible = false;
                    }
                }

                if visible {
                    visibility.push((v1,v2));
                    visibility.push((v2,v1));
                }
            }
        }

        for wall in visible_walls {
            let verts = self.walls[wall];
            visibility.push((verts.1, verts.0));
            visibility.push(verts);
        }

        VisibilityGraph {
            obstacle_map: self.clone(),
            color_cache: area_of_play,
            edges: visibility,
            scale: self.scale
        }
    }

    fn inflate(&self, radius: f64) -> Self
    {
        let image = self.as_image();
        let mut grey_image = GrayImage::new(image.width(), image.height());

        for x in 0..image.width() {
            for y in 0..image.height() {
                if *image.get_pixel(x, y) != Rgb([0u8,0u8,0u8])
                {
                    grey_image.put_pixel(x,y, Luma([255]));
                }
            }
        }

        let radius_u32 = radius.round() as u32;
        let mut inflated_image = GrayImage::new(image.width(), image.height());

        if radius_u32 > image.width()
        {
            // TODO(arjo): Fix inflation if radius is bigger than map.
            return Self::new();
        }

        for x in radius_u32..image.width() - radius_u32 {
            for y in radius_u32..image.height() - radius_u32 {
                for dx in (-radius as i32)..(radius as i32) {
                    for dy in (-radius as i32)..(radius as i32) {
                        if *image.get_pixel((x as i32 + dx) as u32, (y as i32 + dy) as u32 ) != Rgb([0u8,0u8,0u8])
                        {
                            inflated_image.put_pixel(x,y, Luma([255]));
                        }
                    }
                }
            }
        }

        let contours = find_contours::<u32>(&inflated_image);

        let mut vertices = vec!();
        let mut walls = vec!();

        // Find corners
        for contour in contours {
            let mut prev_point: Option<Point<u32>> = None;
            let mut curr_direction:  Option<Vector2<f64>> = None;

            let num_points = contour.points.len();
            if num_points < 2 {
                // Can't form a wall.
                continue;
            }

            let last_point = contour.points[num_points-1];

            let verts_before_addition = vertices.len();

            for vertex in contour.points {
                if let Some(pt) = prev_point {
                    let prev_pt = Vector2::<f64>::new(pt.x as f64, pt.y as f64);
                    let pt = Vector2::<f64>::new(vertex.x as f64, vertex.y as f64);
                    let dir_vec = (pt - prev_pt).normalize();

                    if let Some(direction) = curr_direction {
                        if (1f64 - dir_vec.dot(&direction)).abs() > 1e-2 {
                            // Corner detected
                            vertices.push((vertex.x as f64, vertex.y as f64));
                            curr_direction = Some(dir_vec);
                        }
                    }
                    else {
                        curr_direction = Some(dir_vec);
                    }
                }
                else
                {
                    vertices.push((vertex.x as f64, vertex.y as f64));
                }
                prev_point = Some(vertex);
            }
            if num_points > 1 {
                vertices.push((last_point.x as f64, last_point.y as f64));
            }

            for i in verts_before_addition..vertices.len()-1 {
                walls.push((i, i+1));
            }
        }

        Self {
            vertices,
            walls,
            scale: self.scale
        }
    }

    fn wall_id_by_location(&self) -> HashMap<(u32, u32), usize>
    {
        let (max_x, max_y) = self.get_upper_bounds();
        let (min_x, min_y) = self.get_lower_bounds();
        let mut hashmap = HashMap::new();
        for wall_id in 0..self.walls.len() {
            let (v1, v2) = self.walls[wall_id];
            let (start_x, start_y) = self.to_image_coordinates(self.vertices[v1], (min_x, min_y));
            let (end_x, end_y) = self.to_image_coordinates(self.vertices[v2], (min_x, min_y));
            let start = (start_x as i32, start_y as i32);
            let end = (end_x as i32, end_y as i32);
            for (x, y) in Bresenham::new(start, end) {
                hashmap.insert((x as u32, y as u32), wall_id);
            }
        }
        hashmap
    }

    fn as_image(&self) -> RgbImage
    {
        let (max_x, max_y) = self.get_upper_bounds();
        let (min_x, min_y) = self.get_lower_bounds();
        let mut new_img = RgbImage::new((max_x - min_x) as u32 + 1, (max_y - min_y) as u32 + 1);

        for (v1, v2) in &self.walls {
            let (start_x, start_y) = self.to_image_coordinates(self.vertices[*v1], (min_x, min_y));
            let (end_x, end_y) = self.to_image_coordinates(self.vertices[*v2], (min_x, min_y));
            let start = (start_x as i32, start_y as i32);
            let end = (end_x as i32, end_y as i32);
            for (x, y) in Bresenham::new(start, end) {
                new_img.put_pixel(x as u32, y as u32,  Rgb([255, 0, 0]));
            }
        }
        for (x, y) in &self.vertices {
            new_img.put_pixel(x.round() as u32, y.round() as u32,  Rgb([0, 255, 0]));
        }
        new_img
    }
}

pub struct RMFPlanner
{
    /// The visibility graph
    visibility_graph: VisibilityGraph,
    /// Routes as saved by Agents. (route_index, next_waypoint)
    agent_cache: HashMap<AgentId, (usize, usize)>,
    /// Routes
    route_list: Vec<Vec<Vec2f>>,
    /// By source and target.
    route_plans_by_location: HashMap<(SpatialHash, SpatialHash), usize>,
    /// Scale of spatial hash
    scale: f64
}

impl RMFPlanner
{
    pub fn from_yaml(yaml_str: &str, inflation: f64, scale: f64) -> Self
    {
        let map = ObstacleMap::load_map(yaml_str, scale);
        let inflated_map = map.inflate(inflation);
        let visibility_graph = inflated_map.visibility_graph();
        Self {
            visibility_graph,
            agent_cache: HashMap::new(),
            route_list: vec!(),
            route_plans_by_location: HashMap::new(),
            scale
        }
    }
}

impl HighLevelPlanner for RMFPlanner
{
    /// Get the desired 
    fn get_desired_velocity(&mut self, agent: &Agent, time: std::time::Duration) -> Option<Vec2f>
    {
        if let Some(path) = self.agent_cache.get(&agent.agent_id) {
            let path_id = path.0;
            let mut waypoint_id = path.1;
            // TODO(arjo): 
            if (agent.position - self.route_list[path_id][waypoint_id]).norm() < 1e-1
                &&  {
                waypoint_id += 1;
            }
            return Some((self.route_list[path_id][waypoint_id] - agent.position).normalize());
        }
        else
        {
            // TODO(arjo): Replan route if agent is outside route.
            //println!("Agent {:?} has deviated too much from the route.", agent.agent_id);
            return None;
        }
    }
    /// Set the target position for a given agent
    fn set_target(&mut self, agent: &Agent, point: Vec2f, tolerance: Vec2f){
        let start_pos = agent.position;
        let start_hash = SpatialHash::new(start_pos.x, start_pos.y, 10f64);

        let end_hash = SpatialHash::new(point.x, point.y, 10f64);
        if let Some(idx) = self.route_plans_by_location.get(&(start_hash, end_hash)) {
            self.agent_cache.insert(agent.agent_id, (*idx, 0));
        }
        else
        {
            let result = self.visibility_graph.plan_route((start_pos.x, start_pos.y), (point.x, point.y));
            if let Some(result) = result
            {
                self.route_plans_by_location.insert((start_hash, end_hash), self.route_list.len());
                self.route_list.push(result);
            }
            else
            {
                println!("Failed to find contiguous path between source and target");
            }
        }
    }
    /// Remove an agent
    fn remove_agent_id(&mut self, agent: AgentId) {
        self.agent_cache.remove(&agent);
    }
}