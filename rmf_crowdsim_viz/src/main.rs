use std::sync::{Arc, Mutex};
use rmf_crowdsim::*;
use nannou::prelude::*;

struct NoMap {}

impl Map for NoMap
{
    fn get_occupancy(&self, _pt :Point) -> Option<bool>
    {
        return Some(true);
    }
}

struct NaiveHighLevelPlan {
    default_vel: Vec2f
}

impl NaiveHighLevelPlan
{
    fn new(default_vel: Vec2f) -> Self
    {
        NaiveHighLevelPlan {
            default_vel: default_vel
        }
    }
}

impl<M: Map> HighLevelPlanner<M> for NaiveHighLevelPlan
{
    fn get_desired_velocity(&mut self, agent: &Agent, time: std::time::Duration) -> Option<Vec2f>
    {
        Some(self.default_vel)
    }

    /// Set the target position for a given agent
    fn set_target(&mut self, agent: &Agent, point: Point, tolerance: Vec2f)
    {
        // For now do nothing
    }

    /// Remove an agent
    fn remove_agent_id(&mut self, agent: AgentId)
    {
        // Do nothing
    }

    fn set_map(&mut self, map: Arc<M>)
    {
        // Do nothing
    }
}

struct NoLocalPlan{}

impl<M: Map> LocalPlanner<M> for NoLocalPlan {
    fn get_desired_velocity(&self, agent: &Agent, recommended_velocity: Vec2f, map: Arc<M>) -> Vec2f
    {
        recommended_velocity
    }
}

struct SimulationModel<M: Map>
{
    crowd_simulation: Simulation<M>
}

/// Setup the model
fn create_crowd_model(_app: &App) -> SimulationModel<NoMap> {
    let map = Arc::new(NoMap{});
    let mut model = SimulationModel::<NoMap>{
        crowd_simulation: Simulation::<NoMap>::new(map)
    };

    let agent_start_positions = vec!(Point::new(100f64,100f64), Point::new(100f64,-100f64));

    let speed = Vec2f::new(1.0f64,1.0f64);
    let high_level_planner = Arc::new(Mutex::new(NaiveHighLevelPlan::new(speed)));
    let local_planner = Arc::new(NoLocalPlan{});

    model.crowd_simulation.add_agents(
        &agent_start_positions,
        high_level_planner,
        local_planner
        );
    model
}


fn main() {
    nannou::app(create_crowd_model)
        .update(update) // rather than `.event(event)`, now we only subscribe to updates
        .simple_window(view)
        .size(400, 400)
        .run();
}

fn update(_app: &App, model: &mut SimulationModel<NoMap>, update: Update) {
    model.crowd_simulation.step(update.since_last);
}

fn view(app: &App, model: &SimulationModel<NoMap>, frame: Frame) {
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
