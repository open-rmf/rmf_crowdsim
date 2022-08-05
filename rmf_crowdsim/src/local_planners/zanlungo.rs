use crate::local_planners::local_planner::LocalPlanner;
use crate::map_representation::map::Map;
use crate::Agent;
use crate::AgentId;
use crate::Vec2f;

use std::collections::HashMap;
use std::sync::Arc;

pub struct Zanlungo {
    agent_scale: f64,
    obstacle_scale: f64,
    reaction_time: f64,
    force_distance: f64,
    // TODO Set these properties in a hashmap so we can have diff agents
    agent_mass: f64,
    agent_radius: f64,
    agent_priorities: HashMap<AgentId, f64>,
}

/// Computes the spherical linear interpolation between two vectors
/// the result is (conceptually) (1-t)*p0 + t*p1
/// sinTheta is the sine of the angle between p1 and p1
fn slerp(t: f64, p0: &Vec2f, p1: &Vec2f, sin_theta: f64) -> Vec2f {
    let theta = sin_theta.asin();
    let t0 = ((1f64 - t) * theta).sin() / sin_theta;
    let t1 = (t * theta).sin() / sin_theta;
    return p0 * t0 + p1 * t1;
}

impl Zanlungo {
    pub fn new(
        agent_scale: f64,
        obstacle_scale: f64,
        reaction_time: f64,
        force_distance: f64,
        agent_mass: f64,
        agent_radius: f64,
    ) -> Self {
        Zanlungo {
            agent_scale: agent_scale,
            obstacle_scale: obstacle_scale,
            reaction_time: reaction_time,
            force_distance: force_distance,
            agent_mass: agent_mass,
            agent_radius: agent_radius,
            agent_priorities: HashMap::new(),
        }
    }
    fn time_to_collision(&self, rel_vel: &Vec2f, rel_pos: &Vec2f) -> f64 {
        let a = rel_vel.norm_squared();
        let b = 2f64 * rel_vel.dot(&rel_pos);
        let c = rel_pos.norm_squared() - self.agent_radius * self.agent_radius;

        let discriminant = b * b - 4f64 * a * c;

        // Determine shortest time
        if discriminant < 0f64 {
            return f64::INFINITY;
        }

        let t0 = (-b - discriminant.sqrt()) / (2f64 * a);
        let t1 = (-b + discriminant.sqrt()) / (2f64 * a);

        if (t0 < 0f64 && t1 > 0f64) || (t1 < 0f64 && t0 > 0f64) {
            return 0f64;
        }
        if t0 < t1 && t0 > 0f64 {
            return t0;
        } else if t1 > 0f64 {
            return t1;
        } else {
            return f64::INFINITY;
        }
    }
    /// Computes the time to the first collision
    pub fn compute_tti(&self, current_agent: &Agent, nearby_agents: &Vec<Agent>) -> f64 {
        let mut t_i = f64::INFINITY;
        for n in nearby_agents {
            let rel_vel = n.velocity - current_agent.velocity;
            let rel_pos = n.position - current_agent.position;
            // rel_vel * t + rel_pos

            // Solve for collision here
            let col_time = self.time_to_collision(&rel_vel, &rel_pos);

            if col_time < t_i {
                t_i = col_time;
            }
        }
        t_i
    }

    fn compute_agent_force(&self, agent: &Agent, other_agent: &Agent, t_i: f64) -> Vec2f {
        let def_priority = other_agent.agent_id as f64;
        let other_priority = self
            .agent_priorities
            .get(&other_agent.agent_id)
            .unwrap_or(&def_priority);
        let (weight, my_vel, other_vel) = self.right_of_way_vel(
            &agent.agent_id,
            &agent.velocity,
            &agent.preferred_vel,
            &other_agent.velocity,
            &other_agent.preferred_vel,
            *other_priority,
        );

        let weight = 1f64 - weight;
        let fut_pos = agent.position + my_vel * t_i;
        let other_future_pos = other_agent.position + other_vel * t_i;
        let mut d_ij = fut_pos - other_future_pos;
        let dist = d_ij.norm();
        if weight > 1f64 {
            // Other agent has right of way
            let pref_speed = other_agent.preferred_vel.norm();
            let mut interpolate = true;
            let mut perp_dir = Vec2f::new(0f64, 0f64);

            if pref_speed < 0.0001f64 {
                // he wants to be stationary, accelerate orthogonally to displacement
                let curr_rel_pos = agent.position - other_agent.position;
                perp_dir = Vec2f::new(-curr_rel_pos.y, curr_rel_pos.x);
                if perp_dir.dot(&agent.velocity) < 0f64 {
                    perp_dir = -perp_dir;
                }
            } else {
                // He's moving somewhere, accelerate orthogonally to his preferred direction
                // of travel.
                let pref_dir = other_agent.preferred_vel;
                if pref_dir.dot(&d_ij) > 0f64 {
                    // perpendicular to preferred velocity
                    perp_dir = Vec2f::new(-pref_dir.y, pref_dir.x);
                    if perp_dir.dot(&d_ij) < 0f64 {
                        perp_dir = -perp_dir;
                    }
                } else {
                    interpolate = false;
                }
            }
            // spherical linear interpolation
            if interpolate {
                let mut sin_theta = perp_dir.x * d_ij.y - perp_dir.y * d_ij.x;
                if sin_theta < 0f64 {
                    sin_theta = -sin_theta;
                }
                if sin_theta > 1f64 {
                    sin_theta = 1f64; // clean up numerical error arising from determinant
                }
                d_ij = slerp(weight - 1f64, &d_ij, &perp_dir, sin_theta);
            }
        }

        // Determine if the agents converge
        // TODO(arjo): Use L2 Norm instead
        if dist > (fut_pos - other_future_pos).norm() {
            return Vec2f::new(0f64, 0f64);
        }

        let d_ij_normalized = d_ij.normalize();

        let surface_dist = dist - self.agent_radius * 2f64;

        let mut magnitude = weight * self.agent_scale * (my_vel - other_vel).norm() / t_i;

        if magnitude >= 1e15f64 {
            magnitude = 1e15f64;
        }

        d_ij_normalized * (magnitude * (-surface_dist / self.force_distance).exp())
    }

    // Returns the right of way velocity based on priority
    fn right_of_way_vel(
        &self,
        agent_id: &AgentId,
        agent_vel: &Vec2f,
        self_pref_vel: &Vec2f,
        other_vel: &Vec2f,
        other_pref_vel: &Vec2f,
        other_priority: f64,
    ) -> (f64, Vec2f, Vec2f) {
        // TOOD: Find API for tweaking agent ID
        let def_priority = *agent_id as f64;
        let self_priority = self.agent_priorities.get(agent_id).unwrap_or(&def_priority);
        let right_of_way = (self_priority - other_priority).clamp(-1f64, 1f64);
        if right_of_way < 0f64 {
            let r_2 = (-right_of_way).sqrt(); // right_of_way * right_of_way; // -right_of_way; //
            let vel = agent_vel;
            let other_adjusted_vel = other_vel + r_2 * (other_pref_vel - other_vel);
            return (-r_2, *vel, other_adjusted_vel);
        } else if right_of_way > 0f64 {
            let r_2 = right_of_way.sqrt(); // right_of_way * right_of_way; // right_of_way; //
            let vel = agent_vel + r_2 * (self_pref_vel - agent_vel);
            return (r_2, vel, *other_vel);
        } else {
            return (0f64, *agent_vel, *other_vel);
        }
    }
}

impl<M: Map> LocalPlanner<M> for Zanlungo {
    fn get_desired_velocity(
        &self,
        agent: &Agent,
        nearby_agents: &Vec<Agent>,
        recommended_velocity: Vec2f,
        map: Arc<M>,
    ) -> Vec2f {
        let t_i = self.compute_tti(agent, nearby_agents);

        let mut force = Vec2f::new(0f64, 0f64);
        if t_i != f64::INFINITY {
            for nearby_agent in nearby_agents {
                force += self.compute_agent_force(agent, nearby_agent, t_i);
            }
        }
        recommended_velocity + (force * (1f64 / self.agent_mass))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_time_to_collision_head_on() {
        let zanlungo = Zanlungo::new(1f64, 10f64, 0f64, 5f64, 0.1f64, 4f64);
        let ttc = zanlungo.time_to_collision(&Vec2f::new(1f64, 0f64), &Vec2f::new(-10f64, 0f64));
        assert_eq!(ttc, 6f64);
    }

    #[test]
    fn test_time_to_collision_never_collide() {
        let zanlungo = Zanlungo::new(1f64, 10f64, 0f64, 5f64, 0.1f64, 4f64);
        let ttc = zanlungo.time_to_collision(&Vec2f::new(1f64, 0f64), &Vec2f::new(10f64, 0f64));
        assert_eq!(ttc, f64::INFINITY);
    }
}
