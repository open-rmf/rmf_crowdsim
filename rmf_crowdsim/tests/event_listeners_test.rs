use rmf_crowdsim::local_planners::no_local_plan::NoLocalPlan;
use rmf_crowdsim::spatial_index::location_hash_2d::LocationHash2D;
use rmf_crowdsim::spatial_index::spatial_index::SpatialIndex;
use rmf_crowdsim::source_sink::source_sink::{SourceSink, MonotonicCrowd};
use rmf_crowdsim::*;
use std::sync::{Arc, Mutex};

struct NoMap {}

impl Map for NoMap {
    fn get_occupancy(&self, _pt: Point) -> Option<bool> {
        return Some(true);
    }
}

struct StubHighLevelPlan {
    default_vel: Vec2f,
}

impl StubHighLevelPlan {
    fn new(default_vel: Vec2f) -> Self {
        StubHighLevelPlan {
            default_vel: default_vel,
        }
    }
}

impl<M: Map> HighLevelPlanner<M> for StubHighLevelPlan {
    fn get_desired_velocity(
        &mut self,
        _agent: &Agent,
        _time: std::time::Duration,
    ) -> Option<Vec2f> {
        Some(self.default_vel)
    }

    /// Set the target position for a given agent
    fn set_target(&mut self, _agent: &Agent, _point: Point, _tolerance: Vec2f) {
        // For now do nothing
    }
    /// Remove an agent
    fn remove_agent_id(&mut self, _agent: AgentId) {
        // Do nothing
    }

    fn set_map(&mut self, _map: Arc<M>) {
        // Do nothing
    }
}

#[derive(Debug)]
struct MockEventListener {
    pub added: Vec<AgentId>,
    pub removed: Vec<AgentId>
}

impl MockEventListener {
    pub fn new() -> Self
    {
        MockEventListener {
            added: vec!(),
            removed: vec!()
        }
    }
}

impl EventListener for MockEventListener {

    fn agent_spawned(&mut self, position: Vec2f, agent: AgentId)
    {
        self.added.push(agent);
    }

    /// Called each time an agent is destroyed
    fn agent_destroyed(&mut self, agent: AgentId)
    {
        println!("Removed {}", agent);
        self.removed.push(agent);
    }
}

#[test]
fn test_event_listener_source_sink_api()
{
    let map = Arc::new(NoMap {});
    let velocity = Vec2f::new(1.0f64, 0.0f64);
    let step_size = std::time::Duration::new(1, 0);
    let stub_spatial = spatial_index::location_hash_2d::LocationHash2D::new(
        1000f64,
        1000f64,
        20f64,
        Point::new(-500f64, -500f64),
    );
    let high_level_planner = Arc::new(Mutex::new(StubHighLevelPlan::new(velocity)));
    let local_planner = Arc::new(Mutex::new(NoLocalPlan {}));

    let mut crowd_simulation = Simulation::new(map, stub_spatial);

    let crowd_generator = Arc::new(MonotonicCrowd::new(1f64));

    let source_sink = Arc::new(SourceSink::<NoMap> {
        source: Vec2f::new(0f64, 0f64),
        sink: Vec2f::new(20f64, 0f64),
        radius_sink: 1f64,
        crowd_generator: crowd_generator,
        high_level_planner: high_level_planner,
        local_planner: local_planner,
        agent_eyesight_range: 5f64
    });

    let event_listener = Arc::new(Mutex::new(MockEventListener::new()));

    crowd_simulation.add_event_listener(event_listener.clone());
    crowd_simulation.add_source_sink(source_sink);

    for steps in 0usize..20usize
    {
        assert_eq!(crowd_simulation.agents.len(), steps);
        assert_eq!(event_listener.lock().unwrap().added.len(), steps);
        crowd_simulation.step(step_size);
    }
    for steps in 20usize..25usize
    {
        assert_eq!(crowd_simulation.agents.len(), 19usize);
        assert_eq!(event_listener.lock().unwrap().added.len(), steps);
        assert_eq!(event_listener.lock().unwrap().removed.len(), steps-19usize);
        crowd_simulation.step(step_size);
    }
}