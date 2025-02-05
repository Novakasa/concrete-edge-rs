use std::f32::consts::PI;

use avian3d::prelude::*;
use bevy::prelude::*;
use dynamics::integrator::IntegrationSet;
use serde::{Deserialize, Serialize};

use super::{rewind::RewindState, rig::RigBone, PlayerParams};

pub const CAPSULE_RADIUS: f32 = 0.15;
pub const CAPSULE_HEIGHT: f32 = 4.0 * CAPSULE_RADIUS;
pub const CAST_RADIUS: f32 = 0.8 * CAPSULE_RADIUS;
pub const FRICTION_MARGIN: f32 = 0.98 * 0.98;
pub const GLOBAL_FRICTION: f32 = 1.0;

#[derive(Reflect, Debug, Default, GizmoConfigGroup)]
pub struct PhysicsGizmos;

fn add_results_in_length(dir: Vec3, rhs: Vec3, combined_length: f32) -> Option<Vec3> {
    let dot = dir.dot(rhs);
    let discriminant = dot.powi(2) - rhs.dot(rhs) + combined_length.powi(2);
    if discriminant < 0.0 {
        return None;
    }
    Some((-dot + discriminant.sqrt()) * dir)
}

#[derive(PhysicsLayer, Default)]
pub enum Layer {
    Player,
    #[default]
    Platform,
}

#[derive(Reflect, Debug, Serialize, Deserialize, Default, Clone)]
pub struct PhysicsParams {
    #[serde(default)]
    pub max_speed: f32,
}

#[derive(Reflect, Debug, Serialize, Deserialize, Clone)]
pub struct SpringParams {
    pub ground_spring: PlayerGroundSpring,
    pub ground_spring_crouching: PlayerGroundSpring,
    pub angular_spring: PlayerAngularSpring,
    pub angular_spring_jumping: PlayerAngularSpring,
    pub angular_spring_aerial: PlayerAngularSpring,
}

impl Default for SpringParams {
    fn default() -> Self {
        Self::new()
    }
}

impl SpringParams {
    pub fn new() -> Self {
        Self {
            ground_spring: PlayerGroundSpring::running(),
            ground_spring_crouching: PlayerGroundSpring::crouching(),
            angular_spring: PlayerAngularSpring::running(),
            angular_spring_jumping: PlayerAngularSpring::jumping(),
            angular_spring_aerial: PlayerAngularSpring::aerial(),
        }
    }

    pub fn get_ground_spring(&self, crouching: bool) -> &PlayerGroundSpring {
        if crouching {
            &self.ground_spring_crouching
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

#[derive(Reflect, Debug, Clone, Default, Serialize, Deserialize)]
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
            rest_length: RigBone::legacy_capsule_height() * 0.7 + RigBone::legacy_capsule_radius(),
            min_damping: 1.5,
            stiffness: 15.0,
            ..Self::new()
        }
    }

    pub fn crouching() -> Self {
        Self {
            rest_length: RigBone::legacy_capsule_height() * 0.4 + RigBone::legacy_capsule_radius(),
            min_damping: 2.5,
            stiffness: 25.0,
            ..Self::running()
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

#[derive(Reflect, Debug, Clone, Default, Serialize, Deserialize)]
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
            turn_stiffness: 0.3,
        }
    }

    pub fn jumping() -> Self {
        Self {
            stiffness: 0.3,
            ..Self::running()
        }
    }

    pub fn aerial() -> Self {
        Self {
            stiffness: 0.05,
            damping: 0.05,
            turn_stiffness: 0.05,
        }
    }
}

#[derive(Component, Debug, Default)]
pub struct ExtForce(pub Vec3);

#[derive(Component, Debug, Reflect, Default, Clone)]
pub struct GrabState {
    grab_position: Option<Vec3>,
}

#[derive(Debug, Clone)]
pub struct GroundContact {
    pub contact_point: Vec3,
    pub contact_normal: Dir3,
    pub toi: f32,
}

#[derive(Component, Debug)]
pub struct GroundSpring {
    pub cast_dir: Dir3,
    pub contact: Option<GroundContact>,
}

impl Default for GroundSpring {
    fn default() -> Self {
        Self {
            cast_dir: Dir3::NEG_Y,
            contact: None,
        }
    }
}

#[derive(Component, Default)]
pub struct GroundState {
    pub input_dir: Vec2,
    pub slipping: bool,
    pub jumping: bool,
    pub crouching: bool,
    pub grabbing: bool,
}

#[derive(Component, Debug, Reflect, Default, Clone)]
pub struct GroundForce {
    pub tangential_force: Vec3,
    pub angle_force: Vec3,
    pub normal_force: Vec3,
    pub slope_force: Vec3,
    pub target_force: Vec3,
}

#[derive(Debug, Reflect, Clone)]
pub struct PredictedContact {
    pub contact_point: Vec3,
    pub contact_normal: Dir3,
    pub toi: f32,
    pub contact_position: Vec3,
    pub contact_velocity: Vec3,
}

#[derive(Component, Debug, Reflect, Default, Clone)]
pub struct AirPrediction {
    pub predicted_contact: Option<PredictedContact>,
}

impl GroundForce {
    pub fn spring_force(&self) -> Vec3 {
        self.tangential_force + self.normal_force
    }
}

pub fn predict_contact(
    mut q_player: Query<(
        &mut AirPrediction,
        &GroundSpring,
        &ExtForce,
        &Position,
        &LinearVelocity,
        &ComputedMass,
    )>,
    spatial_query: SpatialQuery,
    mut gizmos: Gizmos<PhysicsGizmos>,
) {
    for (
        mut air_prediction,
        ground_spring,
        external_force,
        Position(position),
        LinearVelocity(velocity),
        mass,
    ) in q_player.iter_mut()
    {
        if ground_spring.contact.is_some() {
            air_prediction.predicted_contact = None;
            continue;
        }
        let filter = SpatialQueryFilter::from_mask(Layer::Platform);
        let mut test_time = 0.0;
        let ext_acc = external_force.0 * mass.inverse();
        'raycasts: while test_time < 2.0 {
            let origin =
                position.clone() + *velocity * test_time + ext_acc * 0.5 * test_time.powi(2);
            let target_time = test_time + 0.1;
            let target =
                position.clone() + *velocity * target_time + ext_acc * 0.5 * target_time.powi(2);
            let cast_dir = Dir3::new(target - origin).unwrap_or(Dir3::Z);
            gizmos.line(origin, target, Color::WHITE);
            let test_vel = *velocity + ext_acc * (test_time + 0.05);
            let max_toi = (target - origin).length();
            let result = spatial_query.shape_hits(
                &Collider::sphere(RigBone::max_contact_dist()),
                origin,
                Quat::IDENTITY,
                cast_dir,
                6,
                &ShapeCastConfig::default().with_max_distance(max_toi),
                &filter,
            );
            for coll in result {
                let contact_toi = test_time + coll.distance / test_vel.length();
                let contact_vel = test_vel + ext_acc * coll.distance / test_vel.length();
                if contact_vel.dot(coll.normal1) > 0.0 {
                    continue;
                }

                gizmos.sphere(
                    Isometry3d::from_translation(origin + cast_dir * coll.distance),
                    RigBone::max_contact_dist(),
                    Color::WHITE.with_alpha(0.1),
                );
                air_prediction.predicted_contact = Some(PredictedContact {
                    contact_point: coll.point2 + origin + cast_dir * coll.distance,
                    contact_normal: Dir3::new(coll.normal1).unwrap(),
                    toi: contact_toi,
                    contact_position: origin + cast_dir * coll.distance,
                    contact_velocity: contact_vel,
                });
                break 'raycasts;
            }

            test_time += 0.1;
        }
    }
}

pub fn update_landing_prediction(
    mut q_physics: Query<(
        &mut AirPrediction,
        &mut GroundSpring,
        &ExtForce,
        &GroundState,
    )>,
    params: Res<PlayerParams>,
) {
    for (air_prediction, mut ground_spring, external_force, input) in q_physics.iter_mut() {
        if ground_spring.contact.is_some() {
            continue;
        }
        if air_prediction.predicted_contact.is_some() {
            let contact = air_prediction.predicted_contact.clone().unwrap();
            let contact_point = ground_spring.cast_dir * (RigBone::max_contact_dist()) * 0.1;
            // println!("{:?}", contact_point);
            let _spring_force = spring_force(
                &contact.contact_velocity,
                params.springs.get_ground_spring(false),
                contact.contact_normal,
                contact_point,
            );
            // println!("{:?}", spring_force);
            // let normal_force = spring_force.project_onto(contact.contact_normal);
            let normal_force = contact.contact_normal * external_force.0.length() * 0.5;
            let (_target_vel, _slope_force, target_force) = get_target_force(
                &ground_spring,
                &external_force,
                &input,
                &contact.contact_velocity,
                normal_force,
                &params,
            );

            ground_spring.cast_dir = Dir3::new(-(normal_force + target_force)).unwrap();
        }
    }
}

pub fn set_external_force(
    mut q_physics: Query<(&mut ExtForce, &ComputedMass, &ComputedAngularInertia)>,
    gravity: Res<Gravity>,
) {
    for (mut external_force, mass, _inertia) in q_physics.iter_mut() {
        external_force.0 = gravity.0 * mass.value();
    }
}

pub fn update_forces(
    mut query: Query<(
        &mut ExternalForce,
        &mut ExternalTorque,
        &Position,
        &Rotation,
        &LinearVelocity,
        &AngularVelocity,
        &mut GroundSpring,
        &mut GroundState,
        &mut GroundForce,
        &mut AirPrediction,
        &ExtForce,
        &mut GrabState,
    )>,
    shape_cast: SpatialQuery,
    dt: Res<Time<Substeps>>,
    params: Res<PlayerParams>,
) {
    for (
        mut force,
        mut torque,
        Position(position),
        Rotation(quat),
        LinearVelocity(velocity),
        AngularVelocity(angular_vel),
        mut ground_spring,
        mut ground_state,
        mut ground_force,
        mut air_prediction,
        external_force,
        mut grab_state,
    ) in query.iter_mut()
    {
        let filter = SpatialQueryFilter::from_mask(Layer::Platform);
        let cast_dir = ground_spring.cast_dir;
        let capsule_up = *quat * Vec3::Y;
        let capsule_right = *quat * Vec3::X;
        let spring = params.springs.get_ground_spring(ground_state.crouching);
        let angular_spring = params.springs.get_angular_spring(ground_state.jumping);
        let forward_dir = *quat * Vec3::NEG_Z;
        if let Some(coll) = shape_cast.cast_shape(
            &Collider::sphere(CAST_RADIUS),
            position.clone(),
            Quat::IDENTITY,
            Dir3::new_unchecked(cast_dir.try_normalize().unwrap()),
            &ShapeCastConfig {
                max_distance: RigBone::max_contact_dist() - CAST_RADIUS,
                ..Default::default()
            },
            &filter,
        ) {
            air_prediction.predicted_contact = None;
            let normal = Dir3::new(coll.normal1).unwrap();

            let contact_point = coll.point2 + cast_dir * coll.distance;
            ground_spring.contact = Some(GroundContact {
                contact_normal: normal,
                contact_point,
                toi: coll.distance,
            });
            let spring_dir = -contact_point.normalize_or_zero();
            // let spring_dir = from_up;

            let mut spring_force = spring_force(velocity, spring, normal, contact_point);
            if (spring_force + external_force.0).dot(normal.into()) > 0.0 && !ground_state.jumping {
                if velocity.dot(normal.into()) > 0.0 {
                    let max_normal_force = 0.5 * external_force.0.length();
                    let max_spring_force = (max_normal_force - external_force.0.dot(normal.into()))
                        / spring_dir.dot(normal.into());
                    spring_force = spring_force.clamp_length_max(max_spring_force);
                }
            }
            if ground_state.jumping {
                spring_force = 3.5 * spring_dir;
            }
            let grounded = spring_force.length() > 0.0001;
            if !grounded {
                force.clear();
                torque.clear();
                continue;
            }

            let normal_force = spring_force.dot(normal.into()) * normal;
            let friction_force = normal_force.length() * GLOBAL_FRICTION;
            let tangential_spring_force = spring_force - normal_force;

            let (target_vel, slope_force, target_force) = get_target_force(
                &ground_spring,
                &external_force,
                &ground_state,
                velocity,
                normal_force,
                &params,
            );

            // println!("{:?}, {:?}", target_force, normal_force);

            let target_spring_dir = (target_force + normal_force).normalize_or_zero();

            let raw_neg_cast_vec = (-ground_spring.cast_dir.as_vec3()
                + 12.0 * (target_spring_dir - spring_dir) * dt.delta_secs())
            .try_normalize()
            .unwrap();

            let cast_quat = Quat::from_rotation_arc(capsule_up, raw_neg_cast_vec);
            let angle = capsule_up.angle_between(raw_neg_cast_vec);
            // println!("{:?}", (angle, capsule_up, raw_neg_cast_vec, quat));
            ground_spring.cast_dir = if angle < 0.0001 {
                Dir3::new(-raw_neg_cast_vec).unwrap()
            } else {
                Dir3::new(
                    -Quat::IDENTITY.slerp(cast_quat, angle.min(0.3 * PI) / angle) * capsule_up,
                )
                .unwrap()
            };

            let mut target_up = (-external_force.0)
                .normalize_or_zero()
                .lerp(spring_dir, 0.8)
                .normalize_or_zero();
            if ground_state.jumping {
                target_up = Vec3::Y;
            }
            let target_right = target_vel
                .cross(target_up)
                .try_normalize()
                .unwrap_or(forward_dir.cross(target_up).try_normalize().unwrap());
            let angular_spring_torque = angular_spring.stiffness
                * Quat::from_rotation_arc(capsule_up, target_up).to_scaled_axis()
                + (angular_spring.turn_stiffness
                    * Quat::from_rotation_arc(capsule_right, target_right).to_scaled_axis())
                - angular_spring.damping * angular_vel.clone();

            let tangential_angle_force =
                -normal.cross(angular_spring_torque) / (normal.dot(contact_point));

            ground_force.tangential_force =
                (tangential_spring_force + tangential_angle_force).clamp_length_max(friction_force);
            ground_force.slope_force = slope_force;
            ground_state.slipping =
                (tangential_spring_force + tangential_angle_force).length() > friction_force;
            ground_force.normal_force = normal_force;

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
            ground_spring.contact = None;
            ground_state.jumping = false;

            let angular_spring = &params.springs.angular_spring_aerial;

            if let Some(contact) = air_prediction.predicted_contact.clone() {
                let target_up = -ground_spring.cast_dir;
                let target_forward = contact
                    .contact_velocity
                    .reject_from(contact.contact_normal.into());
                let target_right = target_forward
                    .cross(target_up.into())
                    .try_normalize()
                    .unwrap_or(forward_dir.cross(target_up.into()).try_normalize().unwrap());
                let apply_turn = if ground_state.slipping { 0.0 } else { 1.0 };
                let angular_spring_torque = angular_spring.stiffness
                    * Quat::from_rotation_arc(capsule_up, target_up.into()).to_scaled_axis()
                    + apply_turn
                        * (angular_spring.turn_stiffness
                            * Quat::from_rotation_arc(capsule_right, target_right)
                                .to_scaled_axis())
                    - angular_spring.damping * angular_vel.clone();
                force.clear();
                torque.clear();
                torque.apply_torque(angular_spring_torque);
            }
        }

        let anchor = *position + capsule_up * RigBone::legacy_capsule_height() * 0.5;
        if let Some(grab_pos) = grab_state.grab_position.as_ref() {
            let delta = grab_pos - anchor;
            let delta_dir = delta.try_normalize().unwrap_or(Vec3::ZERO);
            let anchor_vel = *velocity + angular_vel.cross(anchor - *position);
            let spring_vel = anchor_vel.dot(delta_dir) * delta_dir;
            let spring_vel_2 = anchor_vel - spring_vel;
            let mut grab_force =
                delta_dir * (delta.length() / (0.8 * RigBone::max_contact_dist())).powi(4) * 1.5
                    - 1.5 * spring_vel
                    - 0.6 * spring_vel_2;
            grab_force = grab_force.clamp_length_max(20.0);
            force.apply_force_at_point(grab_force, grab_pos - *position, Vec3::ZERO);
        } else if ground_state.grabbing {
            grab_state.grab_position =
                Some(anchor + capsule_up * RigBone::legacy_capsule_height() * 0.5);
        }
        if grab_state.grab_position.is_some() && !ground_state.grabbing {
            grab_state.grab_position = None;
        }
    }
}

fn spring_force(
    velocity: &Vec3,
    spring: &PlayerGroundSpring,
    normal: Dir3,
    contact_point: Vec3,
) -> Vec3 {
    let spring_dir = -contact_point.try_normalize().unwrap();
    let spring_vel = velocity.dot(normal.into()) / (spring_dir.dot(normal.into()));
    // println!("{:?}", spring_vel);
    // println!("Time {:?}", coll.time_of_impact);
    let length = contact_point.length();
    let damping = spring
        .max_damping
        .lerp(spring.min_damping, normal.dot(Vec3::Y).abs());
    let target = (-spring.stiffness * (length - spring.rest_length) - damping * spring_vel)
        .min(spring.max_force)
        .max(spring.min_force);
    target * spring_dir
}

fn get_target_force(
    ground_spring: &GroundSpring,
    external_force: &ExtForce,
    input: &GroundState,

    velocity: &Vec3,
    normal_force: Vec3,
    params: &PlayerParams,
) -> (Vec3, Vec3, Vec3) {
    let normal = normal_force.normalize();
    let ext_dir = external_force.0.normalize_or_zero();
    let tangent_plane = normal.cross(-ext_dir).normalize_or_zero();
    let tangent_slope = normal.cross(tangent_plane).normalize_or_zero();
    let tangent_z = if normal.dot(Vec3::X).abs() > 0.9999 {
        Vec3::Z
    } else {
        Vec3::X.cross(normal).normalize_or_zero()
    };
    let tangent_x = -tangent_z.cross(normal);
    let input_tangent = input.input_dir.x * tangent_x + input.input_dir.y * tangent_z;
    // bias upwards on walls
    // input_tangent =
    //     (input_tangent - 0.5 * (1.0 - normal.dot(-ext_dir)) * tangent_slope).normalize();
    let tangent_vel = *velocity - velocity.dot(normal) * normal;

    let target_vel = input_tangent * params.physics.max_speed;
    let denominator = 1.0 - tangent_slope.dot(ext_dir).powi(2);
    let slope_force = if denominator < 1.0e-4 {
        external_force.0
    } else {
        -tangent_slope.dot(ext_dir) * normal_force.dot(ext_dir) * tangent_slope / denominator
    };

    // let slope_force = external_forces - normal.dot(external_forces) * normal;

    let mut target_force = 0.3 * (target_vel - tangent_vel);

    let friction_force_margin = normal_force.length() * GLOBAL_FRICTION * FRICTION_MARGIN;

    if (target_force - slope_force).length() > friction_force_margin * FRICTION_MARGIN {
        target_force = add_results_in_length(
            target_force.normalize_or_zero(),
            -slope_force,
            friction_force_margin * FRICTION_MARGIN,
        )
        .unwrap_or(
            (target_force - slope_force).clamp_length_max(friction_force_margin) + slope_force,
        )
    }
    target_force -= slope_force;

    // ok temporarily don't use the smart clamp, this fixes wallrun contact (because the smart clamp leads to forces inconsistent with normal force)
    let vel_delta = target_vel - tangent_vel;
    let vel_delta_length = vel_delta.length();
    let vel_delta_dir = vel_delta.try_normalize().unwrap_or(Vec3::ZERO);
    target_force = (0.17 * vel_delta_length.powf(2.0) * vel_delta_dir
        + 0.1 * vel_delta_length.powf(0.5).min(1.0) * vel_delta_dir
        - slope_force)
        .clamp_length_max(friction_force_margin);
    (target_vel, slope_force, target_force)
}

pub struct PlayerPhysicsPlugin;

impl Plugin for PlayerPhysicsPlugin {
    fn build(&self, app: &mut App) {
        app.insert_gizmo_config(
            PhysicsGizmos::default(),
            GizmoConfig {
                enabled: false,
                ..Default::default()
            },
        );
        app.add_systems(
            SubstepSchedule,
            (update_forces,)
                .before(IntegrationSet::Velocity)
                .run_if(in_state(RewindState::Playing)),
        );
        app.add_systems(
            FixedPostUpdate,
            (
                set_external_force,
                update_landing_prediction,
                predict_contact,
            )
                .chain()
                .before(PhysicsSet::StepSimulation)
                .run_if(in_state(RewindState::Playing)),
        );
    }
}
