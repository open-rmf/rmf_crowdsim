use crate::spatial_index::spatial_index::SpatialIndex;
use crate::{AgentId, Point};

use std::collections::{HashMap, HashSet};

/// A 2D dense grid with list of agents in each region.
/// This structure allows for faster nearest neighbour and radius search. It is ideal for the crowd
/// simulation use case as it allows for O(1) updates and O(r^2) radius search. It would perform
/// poorly if all the agents were clustered into a single cell. However, this is not a problem as
/// crowd simulation is limited by the density of the cluster (you can't have more than a certain
/// number of people in a small amount of space, even if you are a slave trader or the CEO of
/// the next Budget Airline).
pub struct LocationHash2D{
    data: Vec<HashSet<AgentId>>,
    id_to_index: HashMap<AgentId, usize>,
    id_to_exact_location: HashMap<AgentId, Point>,
    width: f64,
    height: f64,
    /// Resolution of width of each cell
    resolution: f64,
    offset: Point
}

impl LocationHash2D
{
    /// Creates a new LocationHash2D instance
    /// # Arguments
    /// * `width` - Width of the grid (usually in meters)
    /// * `height`- Height of the grid (usually in meters)
    /// * `cell_size` - Each cell will be of `cell_size`x`cell_size` dimensions in whatever units
    /// the width and height are.Agent
    /// * `offset` - The position of the 0,0 cell.
    pub fn new(width: f64, height: f64, cell_size: f64, offset: Point) -> Self
    {
        let mut data_table = vec!();

        for _i in 0..((width/cell_size) as usize)
        {
            for _j in 0..((height/cell_size) as usize)
            {
                data_table.push(HashSet::new())
            }
        }

        Self {
            data: data_table,
            id_to_index: HashMap::new(),
            id_to_exact_location: HashMap::new(),
            width: width,
            height: height,
            resolution: cell_size,
            offset: offset
        }
    }

    /// Returns an index given a point
    fn location_to_index(&self, point: Point) -> Result<usize,String>
    {
        // TODO(arjo): These conversions are not safe.
        let x_idx = ((point - self.offset).x / self.resolution) as usize;
        let y_idx = ((point - self.offset).y / self.resolution) as usize;

        let idx = x_idx * ((self.width/ self.resolution) as usize) + y_idx;

        if idx >= self.data.len()
        {
            return Err("Index out of bounds".to_string());
        }

        Ok(idx)
    }

    fn location_to_xy_signed_idx(&self, point: Point) -> (i64, i64)
    {
        let x_idx = ((point - self.offset).x / self.resolution).floor() as i64;
        let y_idx = ((point - self.offset).y / self.resolution).floor() as i64;
        (x_idx, y_idx)
    }

    fn signed_idx_to_data_idx(&self, x_idx: i64, y_idx: i64) -> Option<usize>
    {
        if x_idx < 0 || y_idx < 0
        {
            return None;
        }

        let idx = (x_idx as usize) * ((self.width/ self.resolution) as usize) + (y_idx as usize);
        if idx >= self.data.len()
        {
            return None;
        }

        Some(idx)
    }

    fn get_neighbours_in_cell(&self, x_idx: i64, y_idx: i64) -> Option<Vec<(Point, AgentId)>>
    {
        let mut agents_in_ring = vec!();
        let idx = self.signed_idx_to_data_idx(x_idx, y_idx);
        match idx
        {
            Some(idx) =>
            {
                println!("Got IDX {}", idx);
                for agent_id in &self.data[idx]
                {
                    agents_in_ring.push((self.id_to_exact_location[&agent_id], agent_id.clone()))
                }
            }
            None =>
            {
                return None;
            }
        }
        return Some(agents_in_ring);
    }

    fn get_bounds(&self, radius: f64, position: Point) -> (i64, i64, i64, i64)
    {
        let right_extrema_pt = Point::new(position.x + radius, position.y);
        let (right_extrema_idx, _) = self.location_to_xy_signed_idx(right_extrema_pt);

        let left_extrema_pt = Point::new(position.x - radius, position.y);
        let (left_extrema_idx, _) = self.location_to_xy_signed_idx(left_extrema_pt);

        let top_extrema_pt = Point::new(position.x, position.y + radius);
        let (_, top_extrema_idx) = self.location_to_xy_signed_idx(top_extrema_pt);

        let bottom_extrema_pt = Point::new(position.x, position.y - radius);
        let (_, bottom_extrema_idx) = self.location_to_xy_signed_idx(bottom_extrema_pt);

        (left_extrema_idx, right_extrema_idx, bottom_extrema_idx, top_extrema_idx)
    }
}

impl SpatialIndex for LocationHash2D
{
    fn add_or_update(&mut self, id: AgentId, position: Point) -> Result<(),String>
    {
        let new_index = self.location_to_index(position);
        if let Err(error_msg) = new_index {
           return Err(error_msg);
        }

        let old_index = self.id_to_index.get(&id);
        if let Some(old_index) = old_index
        {
            let new_index = new_index.unwrap();
            if new_index != *old_index
            {
                self.data[*old_index].remove(&id);
                self.data[new_index].insert(id);
                self.id_to_index.insert(id, new_index);
            }
        }
        else
        {
            self.data[new_index.unwrap()].insert(id);
        }

        self.id_to_exact_location.insert(id, position);
        Ok(())
    }

    fn get_nearest_neighbours(&self, n: usize, position: Point) -> Vec<AgentId>
    {
        // TODO(arjo): This routine does not consider if a point is outside the grid
        // it would return empty list which is wrong.
        let (x_idx, y_idx) = self.location_to_xy_signed_idx(position);

        let mut agents = vec!();
        let mut all_out_of_bounds = false;
        let mut step = 0;
        let mut agents_in_ring = vec!();

        while agents_in_ring.len() < n && !all_out_of_bounds
        {
            // For each step scan items in a ring around the center of `step` radius
            let mut num_out_of_bounds = 0;
            let mut num_scanned_cells = 0;
            // If center don't
            if step == 0
            {
                let neighbours = self.get_neighbours_in_cell(x_idx, y_idx);
                if let Some(neighbours) = neighbours
                {
                    agents_in_ring.extend(&neighbours);
                }
                else
                {
                    num_out_of_bounds+=1;
                }
                num_scanned_cells += 1;
            }
            else
            {
                // Search in a ring pattern
                // Top line
                for i in (x_idx - step)..(x_idx + step)
                {
                    let neighbours = self.get_neighbours_in_cell(i, y_idx+step);
                    if let Some(neighbours) = neighbours
                    {
                        agents_in_ring.extend(&neighbours);
                    }
                    else
                    {
                        num_out_of_bounds+=1;
                    }
                    num_scanned_cells += 1;
                }

                // Bottom line
                for i in (x_idx - step)..(x_idx + step)
                {
                    let neighbours = self.get_neighbours_in_cell(i, y_idx-step);
                    if let Some(neighbours) = neighbours
                    {
                        agents_in_ring.extend(&neighbours);
                    }
                    else
                    {
                        num_out_of_bounds+=1;
                    }
                    num_scanned_cells += 1;
                }

                // Left line
                for i in (y_idx - step)..(y_idx + step)
                {
                    let neighbours = self.get_neighbours_in_cell(x_idx-step, i);
                    if let Some(neighbours) = neighbours
                    {
                        agents_in_ring.extend(&neighbours);
                    }
                    else
                    {
                        num_out_of_bounds+=1;
                    }
                    num_scanned_cells += 1;
                }

                // Right line
                for i in (y_idx - step)..(y_idx + step)
                {
                    let neighbours = self.get_neighbours_in_cell(x_idx+step, i);
                    if let Some(neighbours) = neighbours
                    {
                        agents_in_ring.extend(&neighbours);
                    }
                    else
                    {
                        num_out_of_bounds+=1;
                    }
                    num_scanned_cells += 1;
                }
            }

            if num_out_of_bounds == num_scanned_cells
            {
                all_out_of_bounds = true;
            }
            step += 1;
        }
        agents_in_ring.sort_by(|(a_pos, _a_agent), (b_pos, _b_agent)| {
            let a_dist = (a_pos - position).norm();
            let b_dist = (b_pos - position).norm();
            a_dist.partial_cmp(&b_dist).unwrap()
        });

        for i in 0..std::cmp::min(n, agents_in_ring.len())
        {
            let (_pos, agent) = agents_in_ring[i];
            agents.push(agent);
        }

        agents
    }

    fn get_neighbours_in_radius(&self, radius: f64, position: Point) -> Vec<AgentId>
    {
        let (x_idx, y_idx) = self.location_to_xy_signed_idx(position);

        let mut agents = vec!();
        let mut step = 0;

        let (left_bound, right_bound, bottom_bound, top_bound) = self.get_bounds(radius, position);

        for x_idx in left_bound..=right_bound
        {
            for y_idx in bottom_bound..=top_bound
            {
                let result = self.get_neighbours_in_cell(x_idx, y_idx);
                if let Some(result) = result
                {
                    let inliers = result.iter()
                        .filter(|(agent_pos, _)|{ (agent_pos-position).norm() < radius })
                        .map(|(_, agent_id)| agent_id);
                    agents.extend(inliers);
                }
            }
        }
        agents
    }

    fn remove_agent(&mut self, id: AgentId) {
        let index = self.id_to_index.get(&id);
        if let Some(index) = index
        {
            self.data.get_mut(*index).unwrap().remove(&id);
            self.id_to_exact_location.remove(&id);
            self.id_to_index.remove(&id);
        }
    }
}

#[cfg(test)]
mod tests {
    // Note this useful idiom: importing names from outer (for mod tests) scope.
    use super::*;

    fn naive_knearest_neighbours(k: usize, query_point: Point, points: HashMap<AgentId, Point>) ->
        Vec<AgentId>
    {
        let mut all_points = vec!();
        for (agent_id, point) in points
        {
            all_points.push(((point-query_point).norm(), agent_id));
        }
        all_points.sort_by(|(a_pos, _a_id), (b_pos, _b_id)| a_pos.partial_cmp(&b_pos).unwrap());

        let mut result = vec!();
        for i in 0..std::cmp::min(all_points.len(), k)
        {
            let (_pt, id) = all_points[i];
            result.push(id);
        }
        result
    }

    fn naive_radius_search(radius: f64, query_point: Point, points: HashMap<AgentId, Point>) ->
        Vec<AgentId>
    {
        let mut all_points = vec!();
        for (agent_id, point) in points
        {
            if (point-query_point).norm() < radius
            {
                all_points.push(agent_id);
            }
        }
        all_points
    }

    // Tests single nearest neighbour works.
    // TODO(arjo) : Test should be lot more comprehensive
    #[test]
    fn test_nearest_neighbours()
    {
        let mut location_hash = LocationHash2D::new(10f64, 10f64, 0.5f64, Point::new(0f64,0f64));
        let mut id = 0 as usize;
        let mut naive_nn = HashMap::new();

        // Populate the database
        for x in 0..10
        {
            for y in 0..10
            {
                let p = Point::new(x as f64 + 0.5f64, y as f64 + 0.5f64);
                let res =
                    location_hash.add_or_update(id, p);
                if let Err(_msg) = res {
                    assert_eq!(0, 1);
                }
                naive_nn.insert(id, p);
                id += 1;
            }
        }

        // Query nearest neighbour for a given point
        let neighbours = location_hash.get_nearest_neighbours(1, Point::new(0.6f64, 0.6f64));
        assert_eq!(neighbours.len(), 1);
        assert_eq!(neighbours[0], 0);

        // Query nearest neighbour for a given point
        let neighbours = location_hash.get_nearest_neighbours(4, Point::new(1.7f64, 1.6f64));
        let ground_truth_neighbours =
            naive_knearest_neighbours(4, Point::new(1.7f64, 1.6f64), naive_nn);
        assert_eq!(neighbours, ground_truth_neighbours);
    }

    // Tests multiple nearest neighbours
    #[test]
    fn test_radius_search()
    {
        let mut location_hash = LocationHash2D::new(10f64, 10f64, 0.5f64, Point::new(0f64,0f64));
        let mut id = 0 as usize;
        let mut naive_nn = HashMap::new();

        // Populate the database
        for x in 0..10
        {
            for y in 0..10
            {
                let p = Point::new(x as f64 + 0.5f64, y as f64 + 0.5f64);
                let res =
                    location_hash.add_or_update(id, p);
                if let Err(_msg) = res {
                    assert_eq!(0, 1);
                }
                naive_nn.insert(id, p);
                id += 1;
            }
        }

        let agents_gt = naive_radius_search(1.1, Point::new(4f64,4f64), naive_nn);
        let agents_gt: HashSet<&usize> = HashSet::from_iter(agents_gt.iter());

        let agents = location_hash.get_neighbours_in_radius(1.1, Point::new(4f64,4f64));
        let agents: HashSet<&usize> = HashSet::from_iter(agents.iter());

        assert_eq!(agents, agents_gt);
    }
}
