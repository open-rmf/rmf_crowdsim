use std::sync::{Arc, Mutex};
use rmf_crowdsim::*;
use rmf_crowdsim::local_planners::no_local_plan::NoLocalPlan;
use rmf_crowdsim::spatial_index::spatial_index::SpatialIndex;
use nannou::prelude::*;

struct NoMap {}

impl Map for NoMap
{
    fn get_occupancy(&self, _pt :Point) -> Option<bool>
    {
        return Some(true);
    }
}

struct StubHighLevelPlan {
    default_vel: Vec2f
}

impl StubHighLevelPlan
{
    fn new(default_vel: Vec2f) -> Self
    {
        StubHighLevelPlan {
            default_vel: default_vel
        }
    }
}

impl<M: Map> HighLevelPlanner<M> for StubHighLevelPlan
{
    fn get_desired_velocity(&mut self, _agent: &Agent, _time: std::time::Duration) -> Option<Vec2f>
    {
        Some(self.default_vel)
    }

    /// Set the target position for a given agent
    fn set_target(&mut self, _agent: &Agent, _point: Point, _tolerance: Vec2f)
    {
        // For now do nothing
    }

    /// Remove an agent
    fn remove_agent_id(&mut self, _agent: AgentId)
    {
        // Do nothing
    }

    fn set_map(&mut self, _map: Arc<M>)
    {
        // Do nothing
    }
}


struct StubSpatialIndex{}

impl SpatialIndex for StubSpatialIndex{
    fn add_or_update(&mut self, _index: AgentId, _position: Point)-> Result<(),String>
    {
        Ok(())
    }

    fn get_nearest_neighbours(&self, _n: usize, _position: Point)-> Vec<AgentId>
    {
        vec!()
    }

    fn get_neighbours_in_radius(&self, _radius: f64, _position: Point) -> Vec<AgentId>
    {
        vec!()
    }
}

struct SimulationModel<M: Map, T: SpatialIndex>
{
    crowd_simulation: Simulation<M, T>
}

/// Setup the model
fn create_crowd_model(_app: &App) -> SimulationModel<NoMap, StubSpatialIndex> {
    let map = Arc::new(NoMap{});
    let stub_spatial = StubSpatialIndex{};
    let mut model = SimulationModel::<NoMap, StubSpatialIndex>{
        crowd_simulation: Simulation::<NoMap, StubSpatialIndex>::new(map, stub_spatial)
    };

    let agent_start_positions = vec!(Point::new(100f64,100f64), Point::new(100f64,-100f64));

    let speed = Vec2f::new(1.0f64,1.0f64);
    let high_level_planner = Arc::new(Mutex::new(StubHighLevelPlan::new(speed)));
    let local_planner = Arc::new(NoLocalPlan{});

    let res = model.crowd_simulation.add_agents(
        &agent_start_positions,
        high_level_planner,
        local_planner);
    if let Err(error_message) = res {
        panic!("Failed to add crowd simulation:\n\t{}", error_message);
    }
    model
}


fn main() {
    nannou::app(create_crowd_model)
        .update(update) // rather than `.event(event)`, now we only subscribe to updates
        .simple_window(view)
        .size(400, 400)
        .run();
}

fn update(_app: &App, model: &mut SimulationModel<NoMap, StubSpatialIndex>, update: Update) {
    let res = model.crowd_simulation.step(update.since_last);

    if let Err(error_message) = res
    {
        panic!("Simulation step failed with error:\n\t{}", error_message);
    }
}

fn view(app: &App, model: &SimulationModel<NoMap, StubSpatialIndex>, frame: Frame) {
    // Begin drawing
    let draw = app.draw();

    // Clear the background to blue.
    draw.background().color(CORNFLOWERBLUE);

    for agent in &model.crowd_simulation.agents {
        draw.ellipse()
            .color(PLUM)
            .x(agent.position.x as f32)
            .y(agent.position.y as f32)
            .width(20f32)
            .height(20f32);
    }
    draw.to_frame(app, &frame).unwrap();
}
