use std::f32::consts::PI;

use avian3d::prelude::*;
use bevy::prelude::*;
use dynamics::integrator::IntegrationSet;

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

#[derive(Component, Reflect, Debug, Default)]
pub struct PlayerSpringParams {
    pub ground_spring: PlayerGroundSpring,
    pub ground_spring_jumping: PlayerGroundSpring,
    pub angular_spring: PlayerAngularSpring,
    pub angular_spring_jumping: PlayerAngularSpring,
}

impl PlayerSpringParams {
    pub fn new() -> Self {
        Self {
            ground_spring: PlayerGroundSpring::running(),
            ground_spring_jumping: PlayerGroundSpring::jumping(),
            angular_spring: PlayerAngularSpring::running(),
            angular_spring_jumping: PlayerAngularSpring::jumping(),
        }
    }

    pub fn get_ground_spring(&self, jumping: bool) -> &PlayerGroundSpring {
        if jumping {
            &self.ground_spring_jumping
        } else {
            &self.ground_spring
        }
    }

    pub fn get_angular_spring(&self, jumping: bool) -> &PlayerAngularSpring {
        if jumping {
            &self.angular_spring_jumping
        } else {
            &self.angular_spring
        }
    }
}

#[derive(Reflect, Debug, Clone, Default)]
pub struct PlayerGroundSpring {
    pub rest_length: f32,
    pub stiffness: f32,
    pub min_damping: f32,
    pub max_damping: f32,
    pub max_force: f32,
    pub min_force: f32,
}

impl PlayerGroundSpring {
    pub fn new() -> Self {
        Self {
            max_force: 20.0,
            ..Default::default()
        }
    }

    pub fn running() -> Self {
        Self {
            rest_length: CAPSULE_HEIGHT * 0.7 + CAPSULE_RADIUS,
            min_damping: 1.5,
            stiffness: 15.0,
            ..Self::new()
        }
    }

    pub fn jumping() -> Self {
        Self {
            rest_length: CAPSULE_HEIGHT * 1.4 + CAPSULE_RADIUS,
            min_damping: 1.0,
            stiffness: 15.0,
            ..Self::new()
        }
    }

    fn force(&self, length: f32, vel: f32, normal: Vec3, _dt: f32) -> f32 {
        let damping = self
            .max_damping
            .lerp(self.min_damping, normal.dot(Vec3::Y).abs());
        let target = (-self.stiffness * (length - self.rest_length).min(0.0) - damping * vel)
            .min(self.max_force)
            .max(self.min_force);
        target
    }
}

#[derive(Reflect, Debug, Clone, Default)]
pub struct PlayerAngularSpring {
    pub stiffness: f32,
    pub damping: f32,
    pub turn_stiffness: f32,
}

impl PlayerAngularSpring {
    pub fn running() -> Self {
        Self {
            stiffness: 0.5,
            damping: 0.15,
            turn_stiffness: 0.4,
        }
    }

    pub fn jumping() -> Self {
        Self {
            stiffness: 0.3,
            ..Self::running()
        }
    }
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
    pub external_force: Vec3,
    pub forward_dir: Vec3,
    pub input_dir: Vec2,
    prev_substep_vel: Vec3,
    pub ground_state: PhysicsGroundState,
    pub air_state: PhysicsAirState,
}

#[derive(Debug, Reflect, Default)]
pub struct PhysicsGroundState {
    pub neg_cast_vec: Vec3,
    pub slipping: bool,
    pub jumping: bool,
    pub contact_point: Option<Vec3>,
    pub contact_normal: Option<Vec3>,
    pub tangential_force: Vec3,
    pub slope_force: Vec3,
}

#[derive(Debug, Reflect, Default)]
pub struct PredictedContact {
    pub contact_point: Vec3,
    pub contact_normal: Vec3,
    pub toi: f32,
    pub contact_position: Vec3,
}

#[derive(Debug, Reflect, Default)]
pub struct PhysicsAirState {
    pub predicted_contact: Option<PredictedContact>,
}

impl PhysicsGroundState {
    pub fn new() -> Self {
        Self {
            neg_cast_vec: Vec3::Y,
            ..Default::default()
        }
    }
}

impl PhysicsState {
    pub fn new() -> Self {
        Self {
            forward_dir: Vec3::NEG_Z,
            ground_state: PhysicsGroundState::new(),
            ..Default::default()
        }
    }
}

pub fn predict_contact(
    mut q_player: Query<(&mut PhysicsState, &Position, &LinearVelocity)>,
    spatial_query: SpatialQuery,
    mut gizmos: Gizmos,
) {
    for (mut physics, Position(position), LinearVelocity(velocity)) in q_player.iter_mut() {
        if physics.ground_state.contact_point.is_some() {
            continue;
        }
        let filter = SpatialQueryFilter::from_mask(Layer::Platform);
        let mut test_time = 0.0;
        while test_time < 2.0 {
            let origin = position.clone()
                + *velocity * test_time
                + physics.external_force * 0.5 * test_time.powi(2);
            let target_time = test_time + 0.1;
            let target = position.clone()
                + *velocity * target_time
                + physics.external_force * 0.5 * target_time.powi(2);
            let cast_dir = Dir3::new(target - origin).unwrap();
            gizmos.line(origin, target, Color::WHITE);
            let test_vel = *velocity + physics.external_force * (test_time + 0.05);
            let max_toi = (target - origin).length();
            let result = spatial_query.cast_shape(
                &Collider::sphere(MAX_TOI + CAPSULE_RADIUS),
                origin,
                Quat::IDENTITY,
                cast_dir,
                max_toi,
                false,
                filter.clone(),
            );
            if let Some(coll) = result {
                gizmos.sphere(
                    origin + cast_dir * coll.time_of_impact,
                    Quat::IDENTITY,
                    MAX_TOI + CAPSULE_RADIUS,
                    Color::WHITE.with_alpha(0.1),
                );
                let contact_toi = test_time + coll.time_of_impact / test_vel.length();
                physics.air_state.predicted_contact = Some(PredictedContact {
                    contact_point: coll.point2 + origin + cast_dir * coll.time_of_impact,
                    contact_normal: coll.normal1,
                    toi: contact_toi,
                    contact_position: origin + cast_dir * coll.time_of_impact,
                });
                break;
            }

            test_time += 0.1;
        }
    }
}

pub fn set_external_force(mut q_physics: Query<&mut PhysicsState>, gravity: Res<Gravity>) {
    for mut physics in q_physics.iter_mut() {
        physics.external_force = gravity.0;
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
        &PlayerSpringParams,
        &mut PhysicsState,
        &mut PhysicsDebugInfo,
    )>,
    shape_cast: SpatialQuery,
    dt: Res<Time<Substeps>>,
) {
    for (
        mut force,
        mut torque,
        Position(position),
        Rotation(quat),
        LinearVelocity(velocity),
        AngularVelocity(angular_vel),
        Mass(mass),
        spring_params,
        mut physics_state,
        mut debug,
    ) in query.iter_mut()
    {
        let filter = SpatialQueryFilter::from_mask(Layer::Platform);
        let cast_dir = -physics_state.ground_state.neg_cast_vec;
        let capsule_up = *quat * Vec3::Y;
        let capsule_right = *quat * Vec3::X;
        let spring = spring_params.get_ground_spring(physics_state.ground_state.jumping);
        let angular_spring = spring_params.get_angular_spring(physics_state.ground_state.jumping);
        physics_state.forward_dir = *quat * Vec3::NEG_Z;
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
            physics_state.ground_state.contact_normal = Some(normal);

            let contact_point = coll.point2 + cast_dir * coll.time_of_impact;
            physics_state.ground_state.contact_point = Some(contact_point);
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
                physics_state.ground_state.contact_point = None;
                force.clear();
                torque.clear();
                continue;
            }

            debug.spring_force = spring_force;

            let normal_force = spring_force.dot(normal) * normal;
            debug.normal_force = normal_force;
            let friction_force = normal_force.length() * GLOBAL_FRICTION;
            let tangential_spring_force = spring_force - normal_force;
            debug.tangential_force = tangential_spring_force;

            let tangent_plane = normal.cross(Vec3::Y).normalize_or_zero();
            let tangent_slope = normal.cross(tangent_plane).normalize_or_zero();
            let tangent_z = if normal.dot(Vec3::X).abs() > 0.9999 {
                Vec3::Z
            } else {
                Vec3::X.cross(normal).normalize_or_zero()
            };
            let tangent_x = -tangent_z.cross(normal);
            let input_tangent =
                physics_state.input_dir.x * tangent_x + physics_state.input_dir.y * tangent_z;
            let tangent_vel = *velocity - velocity.dot(normal) * normal;

            debug.tangent_vel = tangent_vel;

            let target_vel = input_tangent * 7.0;
            debug.target_vel = target_vel;
            let ext_dir = physics_state.external_force.normalize_or_zero();
            let denominator = 1.0 - tangent_slope.dot(ext_dir).powi(2);
            let slope_force = if denominator == 0.0 {
                physics_state.external_force
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
            physics_state.prev_substep_vel = velocity.clone();
            debug.target_force = target_force;
            // println!("{:?}, {:?}", target_force, normal_force);

            let target_spring_dir = (target_force + normal_force).normalize_or_zero();

            let raw_neg_cast_vec = (physics_state.ground_state.neg_cast_vec
                + 10.0 * (target_spring_dir - spring_dir) * dt.delta_seconds())
            .try_normalize()
            .unwrap();

            let cast_quat = Quat::from_rotation_arc(capsule_up, raw_neg_cast_vec);
            let angle = capsule_up.angle_between(raw_neg_cast_vec);
            // println!("{:?}", (angle, capsule_up, raw_neg_cast_vec, quat));
            physics_state.ground_state.neg_cast_vec = if angle < 0.0001 {
                raw_neg_cast_vec
            } else {
                Quat::IDENTITY.slerp(cast_quat, angle.min(0.3 * PI) / angle) * capsule_up
            };

            let target_up = spring_dir;
            let target_right = target_vel.cross(target_up).try_normalize().unwrap_or(
                physics_state
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

            let tangential_angle_force =
                -normal.cross(angular_spring_torque) / (normal.dot(contact_point));

            debug.angle_force = tangential_angle_force;
            physics_state.ground_state.tangential_force =
                (tangential_spring_force + tangential_angle_force).clamp_length_max(friction_force);
            physics_state.ground_state.slope_force = slope_force;
            physics_state.ground_state.slipping =
                (tangential_spring_force + tangential_angle_force).length() > friction_force;

            force.clear();
            force.apply_force_at_point(
                normal_force
                    + (tangential_spring_force + tangential_angle_force)
                        .clamp_length_max(friction_force),
                contact_point,
                Vec3::ZERO,
            );
            torque.clear();
            torque
                .apply_torque(angular_spring_torque - contact_point.cross(tangential_angle_force));
        } else {
            debug.grounded = false;
            physics_state.ground_state.neg_cast_vec = capsule_up;
            physics_state.ground_state.contact_point = None;
            force.clear();
            torque.clear();
            if let Some(contact) = physics_state.air_state.predicted_contact.as_ref() {
                physics_state.ground_state.neg_cast_vec =
                    -(contact.contact_point - contact.contact_position).normalize_or_zero();
            }
        }
    }
}

pub struct PlayerPhysicsPlugin;

impl Plugin for PlayerPhysicsPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(
            SubstepSchedule,
            (update_ground_force,).before(IntegrationSet::Velocity),
        );
        app.add_systems(
            PostUpdate,
            (predict_contact, set_external_force).before(PhysicsSet::StepSimulation),
        );
    }
}
