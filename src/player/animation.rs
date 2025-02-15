use avian3d::prelude::*;
use bevy::{
    color::palettes::css::{GREEN, RED},
    prelude::*,
    utils::HashMap,
};

use crate::{
    util::{ik2_positions, SpringValue},
    RigGizmos,
};

use super::{
    physics::{ExtForce, GroundCast, GroundContact, GroundForce},
    rewind::RewindState,
    rig::RigBone,
    Player, PlayerParams,
};

#[derive(Debug, Clone, Default, Reflect)]
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

#[derive(Debug, Clone, Reflect)]
pub struct FootLockInfo {
    time: f32,
    pub pos: Vec3,
}

impl FootLockInfo {
    fn new(pos: Vec3) -> Self {
        Self { time: 0.0, pos }
    }
}

#[derive(Debug, Reflect, Clone)]
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

#[derive(Debug, Clone, Reflect)]
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

#[derive(Debug, Default, Reflect, Clone)]
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
        ground_force: &GroundForce,
        ground_spring: &GroundCast,
        external_force: &ExtForce,
        right_dir: Dir3,
        velocity: &Vec3,
        mass: &ComputedMass,
        dt: f32,
        up_dir: Dir3,
        params: &PlayerParams,
    ) {
        self.cycle_state.increment(dt);
        let contact = contact_point + *position;
        let normal = ground_spring.contact.as_ref().unwrap().normal.as_vec3();

        let right_tangent =
            (right_dir.as_vec3() - right_dir.dot(normal) * normal).normalize_or_zero();
        let tangential_vel = *velocity - velocity.dot(normal) * normal;
        let acceleration =
            (ground_force.tangential_force + ground_force.slope_force) * mass.inverse();
        let lock_duration = 0.06;
        let _travel_duration = lock_duration
            * 4.0.lerp(
                1.5,
                (ground_force.spring_force().length() * 0.5 / external_force.0.length()).min(1.0),
            );
        let travel_duration = lock_duration * 2.5;
        let min_lock = 0.03;
        let max_unlock = 0.2;
        let window_pos_ahead =
            contact + 0.5 * (acceleration * 0.5 * lock_duration + tangential_vel) * lock_duration;
        // gizmos.sphere(window_pos_ahead, Quat::IDENTITY, 0.1, Color::BLACK);
        // println!("{:?}", mass.0);
        let window_pos_behind =
            contact + 0.5 * (acceleration * 0.5 * lock_duration - tangential_vel) * lock_duration;
        let min_step_size = RigBone::legacy_capsule_radius() * 0.5;
        let window_travel_dist = (window_pos_ahead - window_pos_behind).length();
        let ahead_to_contact = (window_pos_ahead - contact).length();
        let slip_vel = if ground_force.slipping {
            -0.5 * ground_force.tangential_force * mass.inverse()
        } else {
            Vec3::ZERO
        };
        // println!("{:?}", slip_vel);

        let i_lock = self.get_lock_candidate(window_pos_ahead);

        let offset_length = 0.5.lerp(0.2, (velocity.length() / params.physics.max_speed).min(1.0))
            * RigBone::legacy_capsule_radius();
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

#[derive(Debug, Default, Reflect, Clone)]
pub struct RigAirState;

#[derive(Component, Debug, Default, Reflect, Clone)]
pub struct ProceduralRigState {
    pub cm: Vec3,
    pub cm_offset: SpringValue<Vec3>,
    pub velocity: Vec3,
    pub hip_pos: Vec3,
    pub neck_pos: Vec3,
    pub ground_state: RigGroundState,
    pub air_state: RigAirState,
    pub hip_forward: Vec3,
    pub grounded: bool,
}

impl ProceduralRigState {
    pub fn get_bone_transforms(
        &self,
        rig_gizmos: &mut Gizmos<RigGizmos>,
    ) -> HashMap<RigBone, Transform> {
        let mut transforms = HashMap::default();
        let mut center_of_mass_remainder = self.cm;
        let torso_forward = (self.hip_forward + 0.5 * Vec3::Y).normalize_or_zero();
        let (pos1, _pos2) = ik2_positions(
            RigBone::LowerBack.length(),
            RigBone::UpperBack.length(),
            self.neck_pos - self.hip_pos,
            -torso_forward,
        );
        let spine_pos = self.hip_pos + pos1;
        let lower_up = (spine_pos - self.hip_pos).normalize_or_zero();
        let hip_offset_dir = lower_up.cross(torso_forward).normalize_or_zero();
        let lower_forward = Vec3::X.cross(lower_up).normalize_or_zero();
        let lower_transform = Transform::from_translation(0.5 * (spine_pos + self.hip_pos))
            .looking_to(lower_forward, lower_up);
        transforms.insert(RigBone::LowerBack, lower_transform);
        center_of_mass_remainder -=
            lower_transform.translation * RigBone::LowerBack.relative_mass();

        let head_pos = self.neck_pos
            + (self.neck_pos - self.hip_pos)
                .normalize_or_zero()
                .lerp(Vec3::Y, 0.8)
                .normalize_or_zero()
                * 0.15;
        transforms.insert(RigBone::Head, Transform::from_translation(head_pos));
        center_of_mass_remainder -= head_pos * RigBone::Head.relative_mass();

        let upper_back_up = (self.neck_pos - spine_pos).normalize_or_zero();
        let upper_forward = Vec3::X.cross(upper_back_up).normalize_or_zero();
        let upper_transform = Transform::from_translation(0.5 * (self.neck_pos + spine_pos))
            .looking_to(upper_forward, upper_back_up);
        transforms.insert(RigBone::UpperBack, upper_transform);
        center_of_mass_remainder -=
            upper_transform.translation * RigBone::UpperBack.relative_mass();

        if self.grounded {
            let mut feet_positions = Vec::new();
            let mut feet_center_of_mass = Vec3::ZERO;
            for (i, state) in self.ground_state.foot_states.iter().enumerate() {
                let lr = if i == 0 { 1.0 } else { -1.0 };
                let offset = hip_offset_dir * 0.3 * lr * RigBone::legacy_capsule_radius();
                let pos = match state {
                    FootState::Locked(info) => info.pos,
                    FootState::Unlocked(info) => info.pos,
                };
                feet_positions.push(pos);

                let (pos1, pos2) = ik2_positions(
                    RigBone::LeftUpperLeg.length(),
                    RigBone::LeftLowerLeg.length(),
                    pos - self.hip_pos - offset,
                    self.hip_forward,
                );

                let knee_pos = self.hip_pos + offset + pos1;
                let foot_pos = self.hip_pos + offset + pos2;

                let (upper_bone, lower_bone) = match i {
                    0 => (RigBone::LeftUpperLeg, RigBone::LeftLowerLeg),
                    1 => (RigBone::RightUpperLeg, RigBone::RightLowerLeg),
                    _ => unreachable!(),
                };

                let upper_up = (self.hip_pos + offset - knee_pos).normalize_or_zero();
                let upper_forward = Vec3::X.cross(upper_up).normalize_or_zero();
                let upper_transform =
                    Transform::from_translation(0.5 * (knee_pos + self.hip_pos + offset))
                        .looking_to(upper_forward, upper_up);
                center_of_mass_remainder -=
                    upper_transform.translation * upper_bone.relative_mass();
                feet_center_of_mass += upper_transform.translation * upper_bone.relative_mass();

                transforms.insert(upper_bone, upper_transform);

                let lower_up = (knee_pos - foot_pos).normalize_or_zero();
                let lower_forward = Vec3::X.cross(lower_up).normalize_or_zero();
                let lower_transform = Transform::from_translation(0.5 * (foot_pos + knee_pos))
                    .looking_to(lower_forward, lower_up);
                center_of_mass_remainder -=
                    lower_transform.translation * lower_bone.relative_mass();
                feet_center_of_mass += lower_transform.translation * lower_bone.relative_mass();
                transforms.insert(lower_bone, lower_transform);
            }

            feet_center_of_mass /= 2.0
                * (RigBone::LeftUpperLeg.relative_mass() + RigBone::RightUpperLeg.relative_mass());
            rig_gizmos.sphere(
                Isometry3d::from_translation(feet_center_of_mass),
                0.5 * RigBone::legacy_capsule_radius(),
                Color::from(GREEN),
            );

            let arms_center_of_mass = center_of_mass_remainder
                / (2.0
                    * (RigBone::LeftUpperArm.relative_mass()
                        + RigBone::RightUpperArm.relative_mass()));
            rig_gizmos.sphere(
                Isometry3d::from_translation(arms_center_of_mass),
                0.5 * RigBone::legacy_capsule_radius(),
                Color::from(RED),
            );
            let shoulder_pos = self.neck_pos - upper_back_up * 0.5 * RigBone::UpperBack.length();
            for (i, opposite_foot_pos) in feet_positions.iter().rev().enumerate() {
                let lr = if i == 0 { 1.0 } else { -1.0 };
                let shoulder_offset_dir = upper_back_up.cross(self.hip_forward).normalize_or_zero();
                let shoulder_offset =
                    shoulder_offset_dir * 0.3 * lr * RigBone::legacy_capsule_radius();
                let arm_offset = shoulder_offset_dir * 1.0 * lr * RigBone::legacy_capsule_radius();
                let (upper_bone, lower_bone) = match i {
                    0 => (RigBone::LeftUpperArm, RigBone::LeftLowerArm),
                    1 => (RigBone::RightUpperArm, RigBone::RightLowerArm),
                    _ => unreachable!(),
                };
                let arm_pos = (*opposite_foot_pos - feet_center_of_mass) * 0.7
                    + arms_center_of_mass
                    + arm_offset
                    + self.hip_forward * 0.2;

                let (pos1, pos2) = ik2_positions(
                    RigBone::LeftUpperArm.length(),
                    RigBone::LeftLowerArm.length(),
                    arm_pos - shoulder_pos - shoulder_offset,
                    -self.hip_forward,
                );
                let elbow_pos = shoulder_pos + pos1 + shoulder_offset;
                let hand_pos = shoulder_pos + pos2 + shoulder_offset;

                let upper_up = (shoulder_pos + shoulder_offset - elbow_pos).normalize_or_zero();
                let upper_forward = Vec3::X.cross(upper_up).normalize_or_zero();
                let upper_transform =
                    Transform::from_translation(0.5 * (elbow_pos + shoulder_pos + shoulder_offset))
                        .looking_to(upper_forward, upper_up);
                transforms.insert(upper_bone, upper_transform);

                let lower_up = (elbow_pos - hand_pos).normalize_or_zero();
                let lower_forward = Vec3::X.cross(lower_up).normalize_or_zero();
                let lower_transform = Transform::from_translation(0.5 * (hand_pos + elbow_pos))
                    .looking_to(lower_forward, lower_up);
                transforms.insert(lower_bone, lower_transform);
            }
        }
        transforms
    }
}

pub fn update_procedural_state(
    mut query: Query<
        (
            &Position,
            &Rotation,
            &mut ProceduralRigState,
            &GroundForce,
            &GroundCast,
            &ExtForce,
            &ComputedMass,
            &LinearVelocity,
        ),
        With<Player>,
    >,
    _spatial_query: SpatialQuery,
    dt_physics: Res<Time<Physics>>,
    dt_real: Res<Time>,
    params: Res<PlayerParams>,
) {
    let dt = dt_real.delta_secs() * dt_physics.relative_speed();
    for (
        Position(position),
        Rotation(quat),
        mut rig_state,
        ground_force,
        ground_spring,
        external_force,
        mass,
        LinearVelocity(velocity),
    ) in query.iter_mut()
    {
        rig_state.cm = *position;
        rig_state.grounded = ground_spring.contact.is_some();
        let up_dir = *quat * Dir3::Y;
        let right_dir = *quat * Dir3::X;
        rig_state.hip_forward = up_dir.cross(right_dir.into());

        if let Some(GroundContact {
            contact_point,
            normal: contact_normal,
            toi: _,
            contact_world: _,
        }) = ground_spring.contact
        {
            rig_state.hip_pos = *position + contact_point * 0.2;
            rig_state.neck_pos = *position + up_dir * RigBone::legacy_capsule_height() * 0.3
                - rig_state.hip_pos
                + *position;
            rig_state.ground_state.update(
                contact_point,
                position,
                ground_force,
                ground_spring,
                external_force,
                right_dir,
                velocity,
                mass,
                dt,
                contact_normal,
                &params,
            );
        } else {
            rig_state.ground_state = RigGroundState::default();
            let feet_pos = -up_dir * RigBone::legacy_capsule_height() * 1.0;
            rig_state.hip_pos = *position + feet_pos * 0.2;
            rig_state.neck_pos = *position + up_dir * RigBone::legacy_capsule_height() * 0.3
                - rig_state.hip_pos
                + *position;
        }
    }
}

fn draw_gizmos(
    query: Query<(&GroundCast, &ProceduralRigState), With<Player>>,
    mut rig_gizmos: Gizmos<RigGizmos>,
) {
    for (ground_spring, steps) in query.iter() {
        if ground_spring.contact.is_none() {
            continue;
        }
        for (i, state) in steps.ground_state.foot_states.iter().enumerate() {
            let color = if i == 0 {
                Color::from(RED)
            } else {
                Color::from(GREEN)
            };
            match state {
                FootState::Locked(info) => {
                    rig_gizmos.sphere(
                        Isometry3d::from_translation(info.pos),
                        0.5 * RigBone::legacy_capsule_radius(),
                        color,
                    );
                }
                FootState::Unlocked(info) => {
                    rig_gizmos.sphere(
                        Isometry3d::from_translation(info.pos),
                        0.5 * RigBone::legacy_capsule_radius(),
                        color.with_luminance(0.2),
                    );
                }
            };
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
        app.add_systems(
            Update,
            (
                update_procedural_state.run_if(in_state(RewindState::Playing)),
                draw_gizmos,
            )
                .chain(),
        );
    }
}
