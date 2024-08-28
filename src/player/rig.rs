use avian3d::prelude::*;
use bevy::prelude::*;

use super::{
    physics::{PhysicsState, CAPSULE_HEIGHT, CAPSULE_RADIUS},
    Player,
};

#[derive(Debug, Clone, Default)]
pub struct FootTravelInfo {
    time: f32,
    pub pos: Vec3,
    pos0: Vec3,
    duration: f32,
    target: Option<Vec3>,
}

impl FootTravelInfo {
    fn new(pos: Vec3, duration: f32) -> Self {
        Self {
            time: 0.0,
            pos,
            pos0: pos,
            duration,
            target: None,
        }
    }
}

#[derive(Debug, Clone)]
pub struct FootLockInfo {
    time: f32,
    pos: Vec3,
}

impl FootLockInfo {
    fn new(pos: Vec3) -> Self {
        Self { time: 0.0, pos }
    }
}

#[derive(Debug)]
pub enum FootState {
    Locked(FootLockInfo),
    Unlocked(FootTravelInfo),
}

impl FootState {
    fn locked(pos: Vec3) -> Self {
        Self::Locked(FootLockInfo::new(pos))
    }

    fn unlocked(pos: Vec3, duration: f32) -> Self {
        Self::Unlocked(FootTravelInfo::new(pos, duration))
    }

    fn is_unlocked(&self) -> bool {
        matches!(self, Self::Unlocked(_))
    }
}

impl Default for FootState {
    fn default() -> Self {
        Self::Unlocked(FootTravelInfo::default())
    }
}

#[derive(Component, Debug, Default)]
pub struct ProceduralRigState {
    pub center_of_mass: Vec3,
    pub velocity: Vec3,
    pub hip_pos: Vec3,
    pub foot_states: [FootState; 2],
}

impl ProceduralRigState {
    fn get_lock_candidate(&mut self, window_pos_ahead: Vec3) -> usize {
        match (&self.foot_states[0], &self.foot_states[1]) {
            (FootState::Locked(info0), FootState::Locked(info1)) => {
                if (window_pos_ahead - info0.pos).length() > (window_pos_ahead - info1.pos).length()
                {
                    1
                } else {
                    0
                }
            }
            (FootState::Locked(_), FootState::Unlocked(_)) => 1,
            (FootState::Unlocked(_), FootState::Locked(_)) => 0,
            (FootState::Unlocked(info0), FootState::Unlocked(info1)) => {
                match (info0.target, info1.target) {
                    (Some(_), None) => 0,
                    (None, Some(_)) => 1,
                    (Some(target0), Some(target1)) => {
                        if (info0.pos - target0).length() > (info1.pos - target1).length() {
                            1
                        } else {
                            0
                        }
                    }
                    _ => 1,
                }
            }
        }
    }
}

pub fn update_procedural_steps(
    mut query: Query<
        (
            &Position,
            &Rotation,
            &mut ProceduralRigState,
            &PhysicsState,
            &Mass,
            &LinearVelocity,
        ),
        With<Player>,
    >,
    _spatial_query: SpatialQuery,
    dt_physics: Res<Time<Physics>>,
    dt_real: Res<Time>,
) {
    let dt = dt_real.delta_seconds() * dt_physics.relative_speed();
    for (
        Position(position),
        Rotation(quat),
        mut rig_state,
        move_state,
        mass,
        LinearVelocity(velocity),
    ) in query.iter_mut()
    {
        let up_dir = *quat * Vec3::Y;
        let right_dir = *quat * Vec3::X;
        let _hip_pos = *position - up_dir * CAPSULE_HEIGHT * 0.5;
        if let Some(contact_point) = move_state.contact_point {
            let contact = contact_point + *position;
            let normal = move_state.contact_normal.unwrap();

            let right_tangent = (right_dir - right_dir.dot(normal) * normal).normalize_or_zero();
            let tangential_vel = *velocity - velocity.dot(normal) * normal;
            let acceleration = (move_state.current_force + move_state.ext_force) / mass.0;
            let lock_duration = 0.06;
            let travel_duration = lock_duration * 2.0;
            let window_pos_ahead = contact
                + 0.5 * (acceleration * 0.5 * lock_duration + tangential_vel) * lock_duration;
            let window_pos_behind = contact
                + 0.5 * (acceleration * 0.5 * lock_duration - tangential_vel) * lock_duration;
            let min_step_size = CAPSULE_RADIUS * 0.5;
            let window_travel_dist = (window_pos_ahead - window_pos_behind).length();
            let ahead_to_contact = (window_pos_ahead - contact).length();
            let slip_vel = if move_state.slipping {
                -0.5 * move_state.current_force / mass.0
            } else {
                Vec3::ZERO
            };
            // println!("{:?}", slip_vel);

            let i_lock = rig_state.get_lock_candidate(window_pos_ahead);

            let offset_length = 0.3 * CAPSULE_RADIUS;

            for (i, state) in rig_state.foot_states.iter_mut().enumerate() {
                let lr = if i == 0 { -1.0 } else { 1.0 };
                let foot_offset = right_tangent * lr * offset_length;
                match state {
                    FootState::Locked(info) => {
                        info.pos += slip_vel * dt;
                        let next_lock_pos = contact
                            + (acceleration * (travel_duration + 0.5 * lock_duration)
                                + tangential_vel)
                                * (travel_duration + 0.5 * lock_duration)
                            + foot_offset;
                        let _local_travel_dist = (window_pos_ahead - info.pos).length();
                        let pos_to_next = (info.pos - next_lock_pos).length();
                        let pos_to_contact = (info.pos - contact).length();
                        if pos_to_next > min_step_size
                            && pos_to_contact > window_travel_dist * 0.5
                            && pos_to_contact > 1.3 * ahead_to_contact
                        {
                            *state = FootState::unlocked(info.pos, travel_duration);
                        }
                    }
                    FootState::Unlocked(info) => {
                        if i != i_lock {
                            info.duration += dt;
                        }
                        info.time += dt;
                        if info.time > info.duration {
                            *state = FootState::locked(window_pos_ahead + foot_offset);
                        } else {
                            let t = info.time / info.duration;
                            let lock_time = info.duration - info.time + 0.5 * lock_duration;
                            let target = contact
                                + (acceleration * lock_time + tangential_vel) * lock_time
                                + foot_offset;
                            info.target = Some(target);
                            let floor_pos = info.pos0.lerp(target, smoothstep(t));
                            let lift = up_dir
                                * 0.1
                                * (target - info.pos0).length()
                                * smoothstep(smoothstep(1.0 - 2.0 * (t - 0.5).abs()));
                            info.pos = floor_pos + lift;
                        }
                    }
                }
            }
        }
    }
}

fn smoothstep(t: f32) -> f32 {
    3.0 * t.powi(2) - 2.0 * t.powi(3)
}
