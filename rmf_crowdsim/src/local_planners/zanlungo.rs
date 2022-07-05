use crate::local_planners::local_planner::LocalPlanner;
use crate::map_representation::map::Map;
use crate::Vec2f;
use crate::Agent;
use crate::AgentId;

use std::sync::Arc;
use std::collections::HashMap;

pub struct Zanlungo
{
    agent_scale: f64,
    obstacle_scale: f64,
    reaction_time: f64,
    force_distance: f64,
    // TODO Set these properties in a hashmap so we can have diff agents
    agent_mass: f64,
    agent_radius: f64,
    agent_priorities: HashMap<AgentId, f64>
}

/// Computes the spherical linear interpolation between two vectors
/// the result is (conceptually) (1-t)*p0 + t*p1
/// sinTheta is the sine of the angle between p1 and p1
fn slerp(t: f64, p0: &Vec2f, p1: &Vec2f, sinTheta: f64) -> Vec2f
{
  let theta = sinTheta.asin();
  let t0 = ((1f64 - t) * theta).sin() / sinTheta;
  let t1 = (t * theta).sin() / sinTheta;
  return p0 * t0 + p1 * t1;
}

impl Zanlungo
{
    fn time_to_collision(rel_vel: Vec2f, rel_pos: Vec2f) -> Option<f64>
    {
    }
    /// Computes the time to the first collision
    pub fn compute_tti(&self,
        current_agent: &Agent, nearby_agents: &Vec<Agent>) -> Option<f64>
    {
        let mut t_i = None;
        let mut closest_collision = f64::INFINITY;
        for n in nearby_agents
        {
            let rel_vel = n.velocity - current_agent.velocity;
            let rel_pos = n.position - current_agent.position;
            // rel_vel * t + rel_pos

            // Solve for collision here
            let a = rel_vel.norm_squared();
            let b = - rel_vel.dot(&rel_pos);
            let c = - rel_pos.norm_squared() - 4f64 * self.agent_radius * self.agent_radius;

            let discriminant = b * b - 4f64 * a * c;

            // Determine shortest time
            if discriminant > 0f64 {
                let sol1 = - b - discriminant.sqrt() / (2f64 * a);
                let sol2 = - b + discriminant.sqrt() / (2f64 * a);

                if sol1 > 0f64 && sol2 > 0f64
                {
                    if sol1 < sol2
                    {
                        match t_i {
                            Some(x) => {
                                if x > sol1 {
                                    t_i = Some(sol1);
                                }
                            },
                            None => { t_i = Some(sol1); }
                        }
                    }
                    else
                    {
                        match t_i {
                            Some(x) => {
                                if x > sol2 {
                                    t_i = Some(sol2);
                                }
                            },
                            None => { t_i = Some(sol2); }
                        }
                    }
                }
                else if sol1 > 0f64
                {
                    match t_i {
                        Some(x) => {
                            if x > sol1 {
                                t_i = Some(sol1);
                            }
                        },
                        None => { t_i = Some(sol1); }
                    }
                }
                else if sol2 > 0f64
                {
                    match t_i {
                        Some(x) => {
                            if x > sol2 {
                                t_i = Some(sol2);
                            }
                        },
                        None => { t_i = Some(sol2); }
                    }
                }
            }
        }
        t_i
    }
    /*
    Vector2 Agent::agentForce(const Agent* other, float T_i) const {
  float D = Simulator::FORCE_DISTANCE;
  // Right of way-dependent calculations
  Vector2 myVel = _vel;
  Vector2 hisVel = other->_vel;
  float weight =
      1.f - rightOfWayVel(hisVel, other->_velPref.getPreferredVel(), other->_priority, myVel);

  const Vector2 relVel = myVel - hisVel;

  Vector2 futPos = _pos + myVel * T_i;
  Vector2 otherFuturePos = other->_pos + hisVel * T_i;
  Vector2 D_ij = futPos - otherFuturePos;

  // If the relative velocity is divergent do nothing
  if (D_ij * (_vel - other->_vel) > 0.f) return Vector2(0.f, 0.f);
  float dist = abs(D_ij);
  D_ij /= dist;
  if (weight > 1.f) {
    // Other agent has right of way
    float prefSpeed = other->_velPref.getSpeed();
    Vector2 perpDir;
    bool interpolate = true;
    if (prefSpeed < 0.0001f) {
      // he wants to be stationary, accelerate perpinduclarly to displacement
      Vector2 currRelPos = _pos - other->_pos;
      perpDir.set(-currRelPos.y(), currRelPos.x());
      if (perpDir * _vel < 0.f) perpDir.negate();
    } else {
      // He's moving somewhere, accelerate perpindicularly to his preferred direction
      // of travel.
      const Vector2 prefDir(other->_velPref.getPreferred());
      if (prefDir * D_ij > 0.f) {
        // perpendicular to preferred velocity
        perpDir.set(-prefDir.y(), prefDir.x());
        if (perpDir * D_ij < 0.f) perpDir.negate();
      } else {
        interpolate = false;
      }
    }
    // spherical linear interpolation
    if (interpolate) {
      float sinTheta = det(perpDir, D_ij);
      if (sinTheta < 0.f) {
        sinTheta = -sinTheta;
      }
      if (sinTheta > 1.f) {
        sinTheta = 1.f;  // clean up numerical error arising from determinant
      }
      D_ij.set(slerp(weight - 1.f, D_ij, perpDir, sinTheta));
    }
  }
  dist -= (_radius + other->_radius);
  float magnitude = weight * Simulator::AGENT_SCALE * abs(_vel - other->_vel) / T_i;
  const float MAX_FORCE = 1e15f;
  if (magnitude >= MAX_FORCE) {
    magnitude = MAX_FORCE;
  }
  // float magnitude = weight * Simulator::AGENT_SCALE * abs( myVel - hisVel ) / T_i;
  // 3. Compute the force
  return D_ij * (magnitude * expf(-dist / D));
}*/
    fn compute_agent_force(&self, agent: &Agent, other_agent: &Agent, t_i: f64) -> Vec2f
    {
        let other_priority = self.agent_priorities.get(&other_agent.agent_id).unwrap_or(&0f64);
        let (weight, my_vel, other_vel) = self.rightOfWayVel(
            &agent.agent_id,
            &agent.velocity,
            &agent.preferred_vel,
            &other_agent.velocity,
            &other_agent.preferred_vel,
            *other_priority);

        let weight = 1f64 - weight;
        let fut_pos = agent.position + my_vel * t_i;
        let other_future_pos = other_agent.position + other_vel * t_i;
        let mut d_ij = fut_pos - other_future_pos;
        let dist = d_ij.norm();

        if weight > 1f64 {
            // Other agent has right of way
            let pref_speed = other_agent.preferred_vel.norm();
            let mut interpolate = true;
            let mut perpDir = Vec2f::new(0f64, 0f64);

            if pref_speed < 0.0001f64 {
              // he wants to be stationary, accelerate orthogonally to displacement
              let currRelPos = agent.position - other_agent.position;
              perpDir = Vec2f::new(-currRelPos.y, currRelPos.x);
              if perpDir.dot(&agent.velocity) < 0f64 {
                perpDir = -perpDir;
              }
            } else {
              // He's moving somewhere, accelerate orthogonally to his preferred direction
              // of travel.
              let prefDir = other_agent.preferred_vel;
              if prefDir.dot(&d_ij) > 0f64 {
                // perpendicular to preferred velocity
                perpDir= Vec2f::new(-prefDir.y, prefDir.x);
                if perpDir.dot(&d_ij) < 0f64 {
                    perpDir = -perpDir;
                }
              } else {
                interpolate = false;
              }
            }
            // spherical linear interpolation
            if interpolate
            {
              let mut sinTheta = perpDir.x * d_ij.y - perpDir.y * d_ij.x;
              if sinTheta < 0f64 {
                sinTheta = -sinTheta;
              }
              if sinTheta > 1f64 {
                sinTheta = 1f64;  // clean up numerical error arising from determinant
              }
              d_ij = slerp(weight - 1f64, &d_ij, &perpDir, sinTheta);
            }
        }

        // Determine if the agents converge
        // TODO(arjo): Use L2 Norm instead
        if dist > (fut_pos - other_future_pos).norm()
        {
            return Vec2f::new(0f64, 0f64);
        }

        let d_ij_normalized = d_ij.normalize();

        let surface_dist = dist - self.agent_radius * 2f64;

        let mut magnitude = weight * self.agent_scale * (my_vel - other_vel).norm() / t_i;

        if (magnitude >= 1e15f64) {
            magnitude = 1e15f64;
        }

        d_ij * (magnitude * (-surface_dist / self.force_distance).exp())
    }

    // Returns the right of way velocity based on priority
    fn rightOfWayVel(&self,
        agent_id: &AgentId,
        agent_vel: &Vec2f,
        self_pref_vel: &Vec2f,
        other_vel: &Vec2f,
        other_pref_vel: &Vec2f,
        other_priority: f64) -> (f64, Vec2f, Vec2f) {
      let self_priority = self.agent_priorities.get(agent_id).unwrap_or(&0f64);
      let right_of_way = (self_priority - other_priority).clamp(-1f64, 1f64);
      if right_of_way < 0f64 {
        let r_2 = (-right_of_way).sqrt();  // right_of_way * right_of_way; // -right_of_way; //
        let vel = agent_vel;
        let other_adjusted_vel = other_vel + r_2 * (other_pref_vel - other_vel);
        return (-r_2, *vel, other_adjusted_vel);
      } else if right_of_way > 0f64 {
        let r_2 = right_of_way.sqrt();  // right_of_way * right_of_way; // right_of_way; //
        let vel = agent_vel + r_2 * (self_pref_vel - agent_vel);
        return (r_2, vel, *other_vel);
      } else {
        return (0f64, *agent_vel, *other_vel);
      }
    }
}

impl<M: Map> LocalPlanner<M> for Zanlungo {
    fn get_desired_velocity(&self,
        agent: &Agent, nearby_agents: &Vec<Agent>, recommended_velocity: Vec2f, map: Arc<M>) -> Vec2f
    {
        let t_i = self.compute_tti(agent, nearby_agents);
        let mut force = Vec2f::new(0f64,0f64);
        if let Some(t_i) = t_i
        {
            for nearby_agent in nearby_agents
            {
                force += self.compute_agent_force(agent, nearby_agent, t_i);
            }
        }
        recommended_velocity + (force * (1f64/self.agent_mass))
    }
}
