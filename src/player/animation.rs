use avian3d::prelude::*;
use bevy::prelude::*;

use super::{
    physics::{PhysicsState, CAPSULE_HEIGHT, CAPSULE_RADIUS},
    Player,
};

#[derive(Debug, Reflect, Default, GizmoConfigGroup)]
pub struct RigGizmos;

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
    pub pos: Vec3,
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

#[derive(Debug, Clone)]
enum CycleState {
    Locked(f32),
    Unlocked(f32),
}

impl CycleState {
    fn ensure_locked(&mut self, is_any_locked: bool) {
        match self {
            CycleState::Locked(_) => {
                if !is_any_locked {
                    debug!("setting to unlocked");
                    *self = CycleState::Unlocked(0.0);
                }
            }
            CycleState::Unlocked(_) => {
                if is_any_locked {
                    debug!("setting to locked");
                    *self = CycleState::Locked(0.0);
                }
            }
        }
    }

    fn increment(&mut self, dt: f32) {
        match self {
            Self::Locked(time) => {
                *time += dt;
            }
            Self::Unlocked(time) => {
                *time += dt;
            }
        }
    }

    fn get_unlocked_time(&self) -> f32 {
        match self {
            Self::Locked(_) => 0.0,
            Self::Unlocked(time) => *time,
        }
    }
    fn get_locked_time(&self) -> f32 {
        match self {
            Self::Locked(time) => *time,
            Self::Unlocked(_) => 0.0,
        }
    }
}

impl Default for CycleState {
    fn default() -> Self {
        Self::Locked(0.0)
    }
}

#[derive(Debug, Default)]
pub struct RigGroundState {
    pub foot_states: [FootState; 2],
    cycle_state: CycleState,
    cm_offset: f32,
    torso_extent: f32,
}

impl RigGroundState {
    fn is_any_locked(&self) -> bool {
        self.foot_states.iter().any(|state| !state.is_unlocked())
    }

    fn is_both_locked(&self) -> bool {
        self.foot_states.iter().all(|state| !state.is_unlocked())
    }

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
    fn update(
        &mut self,
        contact_point: Vec3,
        position: &Vec3,
        physics_state: &PhysicsState,
        right_dir: Dir3,
        velocity: &Vec3,
        mass: &ComputedMass,
        dt: f32,
        up_dir: Dir3,
        gizmos: &mut Gizmos<RigGizmos>,
    ) {
        self.cycle_state.increment(dt);
        let contact = contact_point + *position;
        let normal = physics_state.ground_state.contact_normal.unwrap();

        let right_tangent =
            (right_dir.as_vec3() - right_dir.dot(normal) * normal).normalize_or_zero();
        let tangential_vel = *velocity - velocity.dot(normal) * normal;
        let acceleration = (physics_state.ground_state.tangential_force
            + physics_state.ground_state.slope_force)
            * mass.inverse();
        let lock_duration = 0.06;
        let travel_duration = lock_duration * 2.5;
        let min_lock = 0.03;
        let max_unlock = 0.2;
        let window_pos_ahead =
            contact + 0.5 * (acceleration * 0.5 * lock_duration + tangential_vel) * lock_duration;
        // gizmos.sphere(window_pos_ahead, Quat::IDENTITY, 0.1, Color::BLACK);
        // println!("{:?}", mass.0);
        let window_pos_behind =
            contact + 0.5 * (acceleration * 0.5 * lock_duration - tangential_vel) * lock_duration;
        let min_step_size = CAPSULE_RADIUS * 0.5;
        let window_travel_dist = (window_pos_ahead - window_pos_behind).length();
        let ahead_to_contact = (window_pos_ahead - contact).length();
        let slip_vel = if physics_state.ground_state.slipping {
            -0.5 * physics_state.ground_state.tangential_force * mass.inverse()
        } else {
            Vec3::ZERO
        };
        // println!("{:?}", slip_vel);

        let i_lock = self.get_lock_candidate(window_pos_ahead);

        let offset_length = 0.3 * CAPSULE_RADIUS;
        let is_both_locked = self.is_both_locked();
        let is_any_locked = self.is_any_locked();
        let unlocked_time = self.cycle_state.get_unlocked_time();

        for (i, state) in self.foot_states.iter_mut().enumerate() {
            let lr = if i == 0 { -1.0 } else { 1.0 };
            let foot_offset = right_tangent * lr * offset_length;
            match state {
                FootState::Locked(info) => {
                    info.pos += slip_vel * dt;
                    let next_lock_pos = contact
                        + (acceleration * (travel_duration + 0.5 * lock_duration) + tangential_vel)
                            * (travel_duration + 0.5 * lock_duration)
                        + foot_offset;
                    let _local_travel_dist = (window_pos_ahead - info.pos).length();
                    let pos_to_next = (info.pos - next_lock_pos).length();
                    let pos_to_contact = (info.pos - contact).length();
                    if pos_to_next > min_step_size
                        && pos_to_contact > window_travel_dist * 0.5
                        && pos_to_contact > 1.3 * ahead_to_contact
                        && (self.cycle_state.get_locked_time() > min_lock || is_both_locked)
                    {
                        *state = FootState::unlocked(info.pos, travel_duration);
                    }
                }
                FootState::Unlocked(info) => {
                    info.time += dt;
                    if i != i_lock {
                        info.duration += dt;
                    } else {
                        if !is_any_locked {
                            if info.duration - info.time + unlocked_time > max_unlock {
                                info.duration -= dt;
                            }
                        }
                    }
                    if info.time > info.duration {
                        *state = FootState::locked(window_pos_ahead + foot_offset);
                    } else {
                        let t = info.time / info.duration;
                        let lock_time = info.duration - info.time + 0.5 * lock_duration;
                        let target = contact
                            + (0.5 * acceleration * lock_time + tangential_vel) * lock_time
                            + foot_offset;
                        info.target = Some(target);

                        // gizmos.sphere(info.target.unwrap(), Quat::IDENTITY, 0.1, Color::WHITE);
                        // gizmos.arrow(contact, contact + tangential_vel, Color::from(RED));
                        // gizmos.arrow(contact, contact + acceleration, Color::from(BLUE));
                        let floor_pos = info.pos0.lerp(target, smoothstep(t));
                        let lift = up_dir
                            * 0.1
                            * (target - info.pos0).length()
                            * smoothstep(smoothstep(1.0 - 2.0 * (t - 0.5).abs()))
                            * 0.12
                            / travel_duration;
                        info.pos = floor_pos + lift;
                    }
                }
            }
        }

        let is_any_locked = self.is_any_locked();
        self.cycle_state.ensure_locked(is_any_locked);
    }
}

#[derive(Debug, Default)]
pub struct RigAirState;

#[derive(Component, Debug, Default)]
pub struct ProceduralRigState {
    pub center_of_mass: Vec3,
    pub velocity: Vec3,
    pub hip_pos: Vec3,
    pub neck_pos: Vec3,
    pub ground_state: RigGroundState,
    pub air_state: RigAirState,
}

pub fn update_procedural_steps(
    mut query: Query<
        (
            &Position,
            &Rotation,
            &mut ProceduralRigState,
            &PhysicsState,
            &ComputedMass,
            &LinearVelocity,
        ),
        With<Player>,
    >,
    _spatial_query: SpatialQuery,
    dt_physics: Res<Time<Physics>>,
    dt_real: Res<Time>,
    mut gizmos: Gizmos<RigGizmos>,
) {
    let dt = dt_real.delta_secs() * dt_physics.relative_speed();
    for (
        Position(position),
        Rotation(quat),
        mut rig_state,
        move_state,
        mass,
        LinearVelocity(velocity),
    ) in query.iter_mut()
    {
        let up_dir = *quat * Dir3::Y;
        let right_dir = *quat * Dir3::X;

        if let Some(contact_point) = move_state.ground_state.contact_point {
            rig_state.hip_pos = *position
                - up_dir.slerp(Dir3::new(-contact_point).unwrap(), 0.5) * CAPSULE_HEIGHT * 0.15;
            rig_state.ground_state.update(
                contact_point,
                position,
                move_state,
                right_dir,
                velocity,
                mass,
                dt,
                Dir3::new_unchecked(move_state.ground_state.contact_normal.unwrap()),
                &mut gizmos,
            );
        } else {
            rig_state.ground_state = RigGroundState::default();
        }
    }
}

fn smoothstep(t: f32) -> f32 {
    3.0 * t.powi(2) - 2.0 * t.powi(3)
}

fn smoothstart(t: f32) -> f32 {
    t.powi(2)
}

pub struct PlayerAnimationPlugin;
impl Plugin for PlayerAnimationPlugin {
    fn build(&self, app: &mut App) {
        app.init_gizmo_group::<RigGizmos>();
        app.add_systems(Update, update_procedural_steps);
    }
}
