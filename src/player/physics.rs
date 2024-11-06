use std::f32::consts::PI;

use avian3d::prelude::*;
use bevy::prelude::*;

pub const CAPSULE_RADIUS: f32 = 0.2;
pub const CAPSULE_HEIGHT: f32 = 4.0 * CAPSULE_RADIUS;
pub const CAST_RADIUS: f32 = 1.0 * CAPSULE_RADIUS;
pub const MAX_TOI: f32 = CAPSULE_HEIGHT * 1.0;
pub const FRICTION_MARGIN: f32 = 0.98;
pub const GLOBAL_FRICTION: f32 = 1.0;

fn add_results_in_length(dir: Vec3, rhs: Vec3, combined_length: f32) -> Option<Vec3> {
    let dot = dir.dot(rhs);
    let discriminant = dot.powi(2) - rhs.dot(rhs) + combined_length.powi(2);
    if discriminant < 0.0 {
        return None;
    }
    Some((-dot + discriminant.sqrt()) * dir)
}

#[derive(PhysicsLayer)]
pub enum Layer {
    Player,
    Platform,
}

#[derive(Clone)]
enum SpringParams {
    Physical { stiffness: f32, damping: f32 },
    Effective { f: f32, zeta: f32, m: f32 },
}

impl SpringParams {
    fn get_physical(&self) -> Self {
        match self {
            Self::Physical {
                stiffness: _,
                damping: _,
            } => self.clone(),
            Self::Effective { f, zeta, m } => Self::Physical {
                stiffness: (2.0 * PI * f).powi(2) * m,
                damping: zeta * f * m / PI,
            },
        }
    }
}

#[derive(Component, Reflect, Debug, Clone, Default)]
struct SpringValue {
    f: f32,
    zeta: f32,
    m: f32,
    velocity: f32,
    value: f32,
}

impl SpringValue {
    fn update(&mut self, target: f32, dt: f32) -> f32 {
        let k = (2.0 * PI * self.f).powi(2) * self.m;
        let c = self.zeta * self.f * self.m / PI;
        self.velocity += k * (target - self.value) * dt - c * self.velocity * dt;
        self.value += self.velocity * dt;
        self.value
    }
}

#[derive(Component, Reflect, Debug, Resource, Clone, Default)]
#[reflect(Component, Resource)]
pub struct PlayerGroundSpring {
    pub rest_length: f32,
    pub stiffness: f32,
    pub min_damping: f32,
    pub max_damping: f32,
    pub max_force: f32,
    pub min_force: f32,
}

impl PlayerGroundSpring {
    fn force(&mut self, length: f32, vel: f32, normal: Vec3, _dt: f32) -> f32 {
        let damping = self
            .max_damping
            .lerp(self.min_damping, normal.dot(Vec3::Y).abs());
        let target = (-self.stiffness * (length - self.rest_length).min(0.0) - damping * vel)
            .min(self.max_force)
            .max(self.min_force);
        target
    }
}

#[derive(Component, Reflect, Debug, Resource, Clone, Default)]
#[reflect(Component, Resource)]
pub struct PlayerAngularSpring {
    pub stiffness: f32,
    pub damping: f32,
    pub turn_stiffness: f32,
}

#[derive(Component, Reflect, Debug, Default)]
pub struct PhysicsDebugInfo {
    pub grounded: bool,
    pub spring_force: Vec3,
    pub spring_torque: Vec3,
    pub normal_force: Vec3,
    pub tangential_force: Vec3,
    pub shape_toi: f32,
    pub cast_dir: Vec3,
    pub spring_dir: Vec3,
    pub contact_point: Vec3,
    pub position: Vec3,
    pub angle_force: Vec3,
    pub ground_normal: Vec3,
    pub tangent_vel: Vec3,
    pub target_vel: Vec3,
    pub target_force: Vec3,
    pub delta_quat: Quat,
}

#[derive(Component, Reflect, Debug, Default)]
pub struct PhysicsState {
    pub input_dir: Vec3,
    pub spring_height: f32,
    prev_vel: Vec3,
    prev_angular_force: Vec3,
    prev_target_force: Vec3,
    pub neg_cast_vec: Vec3,
    pub slipping: bool,
    pub contact_point: Option<Vec3>,
    pub contact_normal: Option<Vec3>,
    pub current_force: Vec3,
    pub ext_force: Vec3,
    pub forward_dir: Vec3,
}

impl PhysicsState {
    pub fn new() -> Self {
        Self {
            neg_cast_vec: Vec3::Y,
            forward_dir: Vec3::NEG_Z,
            ..Default::default()
        }
    }
}

pub fn update_ground_force(
    mut query: Query<(
        &mut ExternalForce,
        &mut ExternalTorque,
        &Position,
        &Rotation,
        &LinearVelocity,
        &AngularVelocity,
        &Mass,
        &mut PlayerGroundSpring,
        &PlayerAngularSpring,
        &mut PhysicsState,
        &mut PhysicsDebugInfo,
    )>,
    shape_cast: SpatialQuery,
    dt: Res<Time<Substeps>>,
    gravity: Res<Gravity>,
) {
    for (
        mut force,
        mut torque,
        Position(position),
        Rotation(quat),
        LinearVelocity(velocity),
        AngularVelocity(angular_vel),
        Mass(mass),
        mut spring,
        angular_spring,
        mut move_state,
        mut debug,
    ) in query.iter_mut()
    {
        let external_forces = gravity.0;
        let ext_dir = external_forces.normalize_or_zero();
        let filter = SpatialQueryFilter::from_mask(Layer::Platform);
        let cast_dir = -move_state.neg_cast_vec;
        let capsule_up = *quat * Vec3::Y;
        let capsule_right = *quat * Vec3::X;
        move_state.forward_dir = *quat * Vec3::NEG_Z;
        if let Some(coll) = shape_cast.cast_shape(
            &Collider::sphere(CAST_RADIUS),
            position.clone(),
            Quat::IDENTITY,
            Dir3::new_unchecked(cast_dir.try_normalize().unwrap()),
            MAX_TOI,
            false,
            filter.clone(),
        ) {
            let normal = coll.normal1;
            debug.ground_normal = normal;
            move_state.contact_normal = Some(normal);

            let contact_point = coll.point2 + cast_dir * coll.time_of_impact;
            move_state.contact_point = Some(contact_point);
            let spring_dir = -contact_point.normalize_or_zero();
            // let spring_dir = from_up;
            debug.contact_point = contact_point;
            debug.shape_toi = coll.time_of_impact;
            debug.cast_dir = cast_dir;

            let spring_vel = -velocity.dot(normal) / (-spring_dir.dot(normal));
            // println!("Time {:?}", coll.time_of_impact);
            let spring_force = spring.force(
                contact_point.length(),
                spring_vel,
                normal,
                dt.delta_seconds(),
            ) * spring_dir;
            let grounded = spring_force.length() > 0.0001;
            debug.grounded = grounded;
            if !grounded {
                move_state.contact_point = None;
                force.clear();
                torque.clear();
                continue;
            }

            debug.spring_force = spring_force;

            let normal_force = spring_force.dot(normal) * normal;
            debug.normal_force = normal_force;
            let friction_force = normal_force.length() * GLOBAL_FRICTION;
            let tangential_force = spring_force - normal_force;
            debug.tangential_force = tangential_force;

            let tangent_plane = normal.cross(Vec3::Y).normalize_or_zero();
            let tangent_slope = normal.cross(tangent_plane).normalize_or_zero();
            let tangent_z = if normal.dot(Vec3::X).abs() > 0.9999 {
                Vec3::Z
            } else {
                Vec3::X.cross(normal).normalize_or_zero()
            };
            let tangent_x = -tangent_z.cross(normal);
            let input_tangent =
                move_state.input_dir.x * tangent_x + move_state.input_dir.z * tangent_z;
            let tangent_vel = *velocity - velocity.dot(normal) * normal;

            debug.tangent_vel = tangent_vel;

            let target_vel = input_tangent * 7.0;
            debug.target_vel = target_vel;
            let denominator = 1.0 - tangent_slope.dot(ext_dir).powi(2);
            let slope_force = if denominator == 0.0 {
                external_forces
            } else {
                -tangent_slope.dot(ext_dir) * normal_force.dot(ext_dir) * tangent_slope
                    / denominator
            };

            // let slope_force = external_forces - normal.dot(external_forces) * normal;

            let mut target_force = 0.3 * (target_vel - tangent_vel);
            let tangent_contact = contact_point - contact_point.dot(normal) * normal;
            if target_vel.length() < 0.0001 && false {
                let max_stopping_force =
                    0.5 * mass * tangent_vel.length_squared() / tangent_contact.length();
                target_force = target_force.clamp_length_max(1.0 * max_stopping_force);
            }

            /*
            let goal_pos = if tangent_vel.length() < 0.0001 {
                tangent_contact
            } else {
                *position + target_vel
            };
            let mut target_force = 2.5 * (goal_pos - *position) - 4.5 * tangent_vel;
            */

            if (target_force - slope_force).length() > friction_force * FRICTION_MARGIN {
                target_force = add_results_in_length(
                    target_force.normalize_or_zero(),
                    -slope_force,
                    friction_force * FRICTION_MARGIN,
                )
                .unwrap_or(target_force)
            }
            target_force -= slope_force;
            move_state.prev_vel = velocity.clone();
            debug.target_force = target_force;
            // println!("{:?}, {:?}", target_force, normal_force);

            let target_spring_dir = (target_force + normal_force).normalize_or_zero();

            let raw_neg_cast_vec = (move_state.neg_cast_vec
                + 10.0 * (target_spring_dir - spring_dir) * dt.delta_seconds())
            .try_normalize()
            .unwrap();

            let cast_quat = Quat::from_rotation_arc(capsule_up, raw_neg_cast_vec);
            let angle = capsule_up.angle_between(raw_neg_cast_vec);
            // println!("{:?}", (angle, capsule_up, raw_neg_cast_vec, quat));
            move_state.neg_cast_vec = if angle < 0.0001 {
                raw_neg_cast_vec
            } else {
                Quat::IDENTITY.slerp(cast_quat, angle.min(0.3 * PI) / angle) * capsule_up
            };

            let target_up = spring_dir;
            let target_right = target_vel.cross(target_up).try_normalize().unwrap_or(
                move_state
                    .forward_dir
                    .cross(target_up)
                    .try_normalize()
                    .unwrap(),
            );
            let delta_quat = Quat::from_rotation_arc(capsule_up, target_up);

            debug.delta_quat = delta_quat;
            let angular_spring_torque = angular_spring.stiffness
                * Quat::from_rotation_arc(capsule_up, target_up).to_scaled_axis()
                + (angular_spring.turn_stiffness
                    * Quat::from_rotation_arc(capsule_right, target_right).to_scaled_axis())
                - angular_spring.damping * angular_vel.clone();

            debug.spring_torque = angular_spring_torque;

            let angle_force = -normal.cross(angular_spring_torque) / (normal.dot(contact_point));

            debug.angle_force = angle_force;
            move_state.current_force =
                (tangential_force + angle_force).clamp_length_max(friction_force);
            move_state.ext_force = slope_force;
            move_state.slipping = (tangential_force + angle_force).length() > friction_force;

            force.clear();
            force.apply_force_at_point(
                normal_force + (tangential_force + angle_force).clamp_length_max(friction_force),
                contact_point,
                Vec3::ZERO,
            );
            torque.clear();
            torque.apply_torque(angular_spring_torque - contact_point.cross(angle_force));
        } else {
            debug.grounded = false;
            move_state.neg_cast_vec = capsule_up;
            move_state.contact_point = None;
            force.clear();
            torque.clear();
        }
    }
}
