use std::collections::HashMap;
use std::sync::{Arc, Mutex};
pub extern crate nalgebra as na;
use na::Vector2;

pub mod highlevel_planners;
pub mod local_planners;
pub mod source_sink;
pub mod spatial_index;
pub mod rmf;

mod util;

pub use crate::highlevel_planners::highlevel_planners::HighLevelPlanner;
pub use crate::local_planners::local_planner::LocalPlanner;
use crate::source_sink::source_sink::SourceSink;
pub use crate::spatial_index::spatial_index::SpatialIndex;
use crate::util::registry::Registry;

/// Listen to events. This is how an external application should interface
/// with the simulator.
pub trait EventListener {
    /// Called each time an agent is spawned
    /// TODO(arjo): Is it worth doing this or using a single function call
    /// at the end.
    fn agent_spawned(&mut self, position: Vec2f, agent: AgentId);

    /// Called each time an agent is destroyed
    fn agent_destroyed(&mut self, agent: AgentId);
}

/// Agent  ID
pub type AgentId = usize;

/// Point
/// TODO(arjo) Make generic so we support lifts and multilevels
pub type Point = Vector2<f64>;

/// 2-vector
pub type Vec2f = Vector2<f64>;

/// Data representing an individual agent
#[derive(Clone, Copy, Debug)]
pub struct Agent {
    /// Unique Agent ID
    pub agent_id: AgentId,
    /// Position of a point
    pub position: Point,
    /// Orientation of agent
    pub orientation: f64,
    /// Velocity of agent
    pub velocity: Vector2<f64>,
    /// Preferred velocity
    preferred_vel: Vec2f,
    /// Angular velocity of agent
    pub angular_vel: f64,
    /// Eyesight range: How far each individual agent can "see"
    /// TODO(arjo): Replace with "Agent Property" hashmap.
    pub eyesight_range: f64,
}

/// A representation of a simulation session
pub struct Simulation<T: SpatialIndex> {
    /// List of all active agents
    pub agents: HashMap<AgentId, Agent>,
    /// List of all sources and sink.
    source_sinks: Registry<Arc<SourceSink>>,
    /// Spatial Index. Used internally to
    spatial_index: T,
    /// High level planning
    high_level_planner: HashMap<AgentId, Arc<Mutex<dyn HighLevelPlanner>>>,
    /// Local avoidance strategy
    local_planner: HashMap<AgentId, Arc<Mutex<dyn LocalPlanner>>>,
    /// Simulation time
    sim_time: std::time::Duration,
    /// Get last allocated agent id
    last_alloc_agent_id: usize,
    /// Update buffer: Ideally we would not need this if
    /// I had done a better job of the update loop
    update_buffer: HashMap<AgentId, StateUpdateBuffer>,
    /// Event listeners
    event_listeners: Registry<Arc<Mutex<dyn EventListener>>>,
    /// Contains the correspondence between agents and their source/sinks
    source_sink_agent_correspondence: HashMap<AgentId, usize>,
}

/// Internal state update buffer structure
struct StateUpdateBuffer {
    new_vel: Vec2f,
    new_pos: Vec2f,
    updated: bool,
}

impl<T: SpatialIndex> Simulation<T> {
    /// Create a new simulation environment
    pub fn new(spatial_index: T) -> Self {
        Self {
            agents: HashMap::new(),
            source_sinks: Registry::new(),
            spatial_index: spatial_index,
            high_level_planner: HashMap::new(),
            local_planner: HashMap::new(),
            sim_time: std::time::Duration::new(0, 0),
            last_alloc_agent_id: 0,
            update_buffer: HashMap::new(),
            event_listeners: Registry::new(),
            source_sink_agent_correspondence: HashMap::new(),
        }
    }

    /// Adds a group of agents
    pub fn add_agents(
        &mut self,
        spawn_positions: &Vec<Point>,
        high_level_planner: Arc<Mutex<dyn HighLevelPlanner>>,
        local_planner: Arc<Mutex<dyn LocalPlanner>>,
        agent_eyesight_range: f64,
    ) -> Result<Vec<AgentId>, String> {
        let mut res = Vec::<AgentId>::new();
        for x in spawn_positions {
            let agent_id = self.last_alloc_agent_id;
            self.last_alloc_agent_id += 1;
            self.high_level_planner
                .insert(agent_id, high_level_planner.clone());
            self.local_planner.insert(agent_id, local_planner.clone());
            self.agents.insert(
                agent_id,
                Agent {
                    agent_id: agent_id,
                    position: *x,
                    orientation: 0f64,
                    velocity: Vector2::<f64>::new(0f64, 0f64),
                    preferred_vel: Vector2::<f64>::new(0f64, 0f64),
                    angular_vel: 0f64,
                    eyesight_range: agent_eyesight_range,
                },
            );
            let success = self.spatial_index.add_or_update(agent_id, *x);
            if let Err(error_message) = success {
                return Err(error_message);
            }
            res.push(agent_id);
            for listener in self.event_listeners.registry.values() {
                listener.lock().unwrap().agent_spawned(*x, agent_id);
            }
        }
        Ok(res)
    }

    /// Adds a source-sink. Returns the id of the source/sink.
    pub fn add_source_sink(&mut self, source_sink: Arc<SourceSink>) -> usize {
        self.source_sinks.add_new_item(source_sink)
    }

    /// Removes a source-sink
    pub fn remove_source_sink(&mut self, id: &usize) {
        // TODO(arjo): Fix this method to remove crowds spawned by
        // registry as well
        self.source_sinks.registry.remove_entry(id);
    }

    /// Adds an event listener
    pub fn add_event_listener(&mut self, event_listener: Arc<Mutex<dyn EventListener>>) -> usize {
        self.event_listeners.add_new_item(event_listener)
    }

    /// removes a set of agents
    pub fn remove_agents(&mut self, agent: AgentId) {
        self.high_level_planner[&agent]
            .lock()
            .unwrap()
            .remove_agent_id(agent);
        self.local_planner[&agent]
            .lock()
            .unwrap()
            .remove_agent(agent);
        self.agents.remove_entry(&agent);
        self.update_buffer.remove_entry(&agent);
        self.source_sink_agent_correspondence.remove_entry(&agent);
        self.spatial_index.remove_agent(agent);
        for listener in self.event_listeners.registry.values() {
            listener.lock().unwrap().agent_destroyed(agent);
        }
    }

    pub fn step(&mut self, dur: std::time::Duration) -> Result<(), String> {
        // Spawn agents using source sinks.
        // TODO(arjo): lots of unessecary allocations going on here to keep
        // the borrow checker happy
        let to_add: Vec<(usize, Arc<SourceSink>, Vec<Vec2f>)> = self
            .source_sinks
            .registry
            .iter()
            .map(|(source_sink_id, source_sink)| {
                let spawn_number = source_sink.crowd_generator.get_number_to_spawn(dur);
                // TODO(arjo): deconflict spawn points.
                let mut agent_spawn_points = vec![];
                //for i in 0..spawn_number {
                if spawn_number > 0 {
                    /// TODO: Remove hard coded constant
                    let neighbours = self.spatial_index.get_neighbours_in_radius(0.4, source_sink.source);
                    if neighbours.len() == 0
                    {
                        agent_spawn_points.push(source_sink.source);
                    }
                }
                //}
                (*source_sink_id, source_sink.clone(), agent_spawn_points)
            })
            .collect();
        let to_add: Vec<(usize, Result<Vec<usize>, String>)> = to_add
            .iter()
            .map(|(source_id, source_sink, agents)| {
                let agents = self.add_agents(
                    &agents,
                    source_sink.high_level_planner.clone(),
                    source_sink.local_planner.clone(),
                    source_sink.agent_eyesight_range,
                );
                (*source_id, agents)
            })
            .collect();

        for (source_id, agents) in to_add {
            if let Ok(agents) = agents {
                for agent in agents {
                    self.source_sink_agent_correspondence
                        .insert(agent, source_id);
                    self.high_level_planner[&agent].lock().unwrap().set_target(
                        &self.agents[&agent],
                        self.source_sinks.registry[&source_id].sink,
                        Vec2f::new(
                            self.source_sinks.registry[&source_id].radius_sink,
                            self.source_sinks.registry[&source_id].radius_sink,
                        ),
                    );
                }
            } else {
                return Err("Failed to add agents from source".to_string());
            }
        }

        // Calculate motion updates
        for agent_id in self.agents.keys() {
            // Safe to unwrap as agent_id definitely exists.
            let mut agent = self.agents.get(agent_id).unwrap().clone();
            // Execute the high level plan
            let mut vel = Vec2f::new(0f64, 0f64);
            if let Some(lock) = self.high_level_planner.get(&agent_id) {
                let result = lock
                    .lock()
                    .unwrap()
                    .get_desired_velocity(&agent, self.sim_time);
                if let Some(res) = result {
                    vel = res;
                    agent.preferred_vel = vel.clone();
                }
            }

            // Execute the local planner
            if let Some(local_planner) = self.local_planner.get(&agent_id) {
                let neighbour_ids = self
                    .spatial_index
                    .get_neighbours_in_radius(agent.eyesight_range, agent.position);

                let neighbours = Vec::from_iter(
                    neighbour_ids
                        .iter()
                        .filter(|neighbor_id| **neighbor_id != *agent_id)
                        .map(|neighbor_id| self.agents.get(neighbor_id).unwrap().clone()),
                );

                vel = local_planner.lock().unwrap().get_desired_velocity(
                    &agent,
                    &neighbours,
                    vel
                );
            }

            // Agent position
            let dx = vel * dur.as_secs_f64();
            let pos = agent.position;
            let new_pos = pos + dx;

            self.update_buffer.insert(
                *agent_id,
                StateUpdateBuffer {
                    new_pos: new_pos,
                    new_vel: vel,
                    updated: true,
                },
            );

            let success = self.spatial_index.add_or_update(*agent_id, new_pos);
            if let Err(error_message) = success {
                return Err(error_message);
            }
        }

        // Commit updates
        for (id, state_update) in self.update_buffer.iter_mut() {
            if !state_update.updated {
                continue;
            }
            let mut agent = self.agents.get_mut(id).unwrap();
            agent.velocity = state_update.new_vel;
            agent.position = state_update.new_pos;
            state_update.updated = false;
        }

        // Remove agents
        let mut to_be_removed: Vec<AgentId> = vec![];
        for (sink_id, source_sink) in self.source_sinks.registry.iter() {
            let candidates_to_be_removed = self
                .spatial_index
                .get_neighbours_in_radius(source_sink.radius_sink, source_sink.sink);
            to_be_removed.extend(candidates_to_be_removed.iter().filter(|candidate| {
                let res = self.source_sink_agent_correspondence.get(candidate);
                matches!(res, Some(x) if *x == *sink_id)
            }));
        }

        for i in to_be_removed {
            self.remove_agents(i);
        }

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    // Note this useful idiom: importing names from outer (for mod tests) scope.
    use super::*;

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

    impl HighLevelPlanner for StubHighLevelPlan {
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
    }

    #[test]
    fn test_step_integration() {
        let velocity = Vec2f::new(1.0f64, 0.0f64);
        let step_size = std::time::Duration::new(1, 0);
        let stub_spatial = spatial_index::location_hash_2d::LocationHash2D::new(
            1000f64,
            1000f64,
            20f64,
            Point::new(-500f64, -500f64),
        );
        let mut crowd_simulation = Simulation::new(stub_spatial);
        let agent_start_positions = vec![Point::new(0f64, 0f64)];
        let high_level_planner = Arc::new(Mutex::new(StubHighLevelPlan::new(velocity)));
        let local_planner = Arc::new(Mutex::new(local_planners::no_local_plan::NoLocalPlan {}));

        assert_eq!(crowd_simulation.agents.len(), 0usize);

        let agents = crowd_simulation.add_agents(
            &agent_start_positions,
            high_level_planner,
            local_planner,
            100f64,
        );
        assert!(matches!(agents, Ok(agent) if agent.len() == 1usize));
        assert_eq!(crowd_simulation.agents.len(), 1usize);

        let res = crowd_simulation.step(step_size);
        assert_eq!(res, Ok(()));
        assert_eq!(crowd_simulation.agents.len(), 1usize);

        assert!((crowd_simulation.agents[&(0u64 as usize)].position - velocity).norm() < 1e-5f64);
    }
}
