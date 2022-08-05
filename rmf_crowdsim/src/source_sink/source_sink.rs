use crate::{Vec2f, AgentId};

trait CrowdGenerator
{
    fn spawn_location() -> Vec2f;
}

trait EventListener
{
    fn agent_spawned(position: Vec2f, agent: AgentId);

    fn agent_destroyed(agent: AgentId);
}

struct SourceSink
{
    source: Vec2f,
    sink: Vec2f,
    radius_sink: f64
}

struct PoissonCrowd
{
    rate: f64
}