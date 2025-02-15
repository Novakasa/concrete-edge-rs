use avian3d::prelude::*;
use bevy::{
    color::palettes::{
        css::{BLUE, GREEN, ORANGE, RED, VIOLET, WHITE, YELLOW},
        tailwind::CYAN_100,
    },
    prelude::*,
};
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

#[derive(Component, Debug, Default, Clone)]
pub struct ExtForce(pub Vec3);

#[derive(Component, Debug, Reflect, Default, Clone)]
pub struct GrabState {
    grab_position: Option<Vec3>,
}

#[derive(Debug, Clone)]
pub struct GroundContact {
    pub contact_point: Vec3,
    pub contact_world: Vec3,
    pub normal: Dir3,
    pub toi: f32,
}

#[derive(Component, Debug, Clone)]
pub struct GroundCast {
    pub cast_dir: Dir3,
    pub contact: Option<GroundContact>,
}

impl Default for GroundCast {
    fn default() -> Self {
        Self {
            cast_dir: Dir3::NEG_Y,
            contact: None,
        }
    }
}

#[derive(Component, Debug, Default, Clone)]
pub struct NaiveInput {
    pub input_dir: Vec2,
    pub jump_pressed: bool,
    pub jump_pressed_time: f32,
    pub crouch_pressed: bool,
    pub grab_pressed: bool,
}

#[derive(Component, Default, Clone, Debug)]
pub struct PlayerInput {
    pub input_dir: Vec2,
    pub target_velocity: Vec3,
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
    pub slipping: bool,
    pub angular_spring_torque: Vec3,
}

#[derive(Debug, Reflect, Clone)]
pub struct PredictedContact {
    pub contact_world: Vec3,
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

pub fn physics_input(
    mut q_input: Query<(&mut PlayerInput, &mut NaiveInput, &GroundCast)>,
    params: Res<PlayerParams>,
) {
    for (mut input, mut naive_input, ground_cast) in q_input.iter_mut() {
        input.input_dir = naive_input.input_dir;
        input.crouching = naive_input.crouch_pressed;
        input.grabbing = naive_input.grab_pressed;
        if let Some(_contact) = ground_cast.contact.as_ref() {
            if naive_input.jump_pressed && naive_input.jump_pressed_time < params.input.prejump_time
            {
                input.jumping = true;
            }
        } else {
            if input.jumping {
                // make this jump input invalid, because we already jumped
                // this could be more robust since the contact that ate the jump could have been very short and thus uneffective
                naive_input.jump_pressed_time += params.input.prejump_time;
            }
            input.jumping = false;
        }
    }
}

pub fn predict_contact(
    mut q_player: Query<(
        &mut AirPrediction,
        &GroundCast,
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
                    contact_world: coll.point2 + origin + cast_dir * coll.distance,
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
    mut q_physics: Query<(&mut AirPrediction, &mut GroundCast, &ExtForce, &PlayerInput)>,
    params: Res<PlayerParams>,
    mut gizmos: Gizmos<PhysicsGizmos>,
) {
    for (air_prediction, mut ground_spring, external_force, input) in q_physics.iter_mut() {
        if ground_spring.contact.is_some() {
            continue;
        }
        if air_prediction.predicted_contact.is_some() {
            let contact = air_prediction.predicted_contact.clone().unwrap();
            if ground_spring.cast_dir.dot(contact.contact_velocity) <= 0.0 {
                ground_spring.cast_dir = Dir3::new(contact.contact_velocity).unwrap();
            }
            let contact_point = ground_spring.cast_dir * (RigBone::max_contact_dist()) * 0.1;
            // println!("{:?}", contact_point);
            let spring_force = spring_force(
                &contact.contact_velocity,
                params.springs.get_ground_spring(false),
                contact.contact_normal,
                contact_point,
            );
            // println!("{:?}", spring_force);
            let normal_force = spring_force.project_onto(contact.contact_normal.into());
            if normal_force.dot(contact.contact_normal.into()) < 1.0e-4 {
                continue;
            }
            gizmos.arrow(
                contact.contact_world,
                contact.contact_world + spring_force,
                YELLOW,
            );
            gizmos.arrow(
                contact.contact_world,
                contact.contact_world + normal_force,
                WHITE,
            );
            // let normal_force = contact.contact_normal * external_force.0.length() * 0.5;
            let (_target_vel, _slope_force, target_force) = get_target_force(
                &external_force,
                &input,
                &contact.contact_velocity,
                normal_force,
                &params,
            );

            ground_spring.cast_dir =
                Dir3::new(-(normal_force + target_force)).unwrap_or(Dir3::NEG_Y);
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

pub fn evaluate_ground_cast(
    mut query: Query<(&mut GroundCast, &Position)>,
    spatial_query: SpatialQuery,
) {
    for (mut ground_cast, Position(pos)) in query.iter_mut() {
        let filter = SpatialQueryFilter::from_mask(Layer::Platform);
        if let Some(coll) = spatial_query.cast_shape(
            &Collider::sphere(CAST_RADIUS),
            pos.clone(),
            Quat::IDENTITY,
            ground_cast.cast_dir,
            &ShapeCastConfig {
                max_distance: RigBone::max_contact_dist() - CAST_RADIUS,
                ..Default::default()
            },
            &filter,
        ) {
            let normal = Dir3::new(coll.normal1).unwrap();
            let contact_point = coll.point2 + ground_cast.cast_dir * coll.distance;
            ground_cast.contact = Some(GroundContact {
                normal,
                contact_point,
                contact_world: pos.clone() + contact_point,
                toi: coll.distance,
            });
        } else {
            ground_cast.contact = None;
        }
    }
}

pub fn evaluate_ground_spring(
    mut query: Query<(
        &mut GroundForce,
        &LinearVelocity,
        &GroundCast,
        &ExtForce,
        &PlayerInput,
    )>,
    params: Res<PlayerParams>,
) {
    for (mut ground_force, velocity, ground_cast, external_force, input) in query.iter_mut() {
        if let Some(contact) = ground_cast.contact.as_ref() {
            let spring = params.springs.get_ground_spring(input.crouching);
            let mut spring_force =
                spring_force(velocity, spring, contact.normal, contact.contact_point);
            if (spring_force + external_force.0).dot(contact.normal.into()) > 0.0 && !input.jumping
            {
                if velocity.dot(contact.normal.into()) > 0.0 {
                    let max_normal_force = 0.5 * external_force.0.length();
                    let max_spring_force = (max_normal_force
                        - external_force.0.dot(contact.normal.into()))
                        / -ground_cast.cast_dir.dot(contact.normal.into());
                    spring_force = spring_force.clamp_length_max(max_spring_force);
                }
            }
            if input.jumping {
                spring_force = -3.5 * contact.contact_point.normalize_or_zero();
                // spring_force = -3.5 * ground_cast.cast_dir.as_vec3();
            }

            ground_force.normal_force = spring_force.dot(contact.normal.into()) * contact.normal;
            ground_force.tangential_force = spring_force - ground_force.normal_force;
        }
    }
}

pub fn update_cast_dir(
    mut query: Query<(
        &mut GroundCast,
        &mut PlayerInput,
        &mut GroundForce,
        &LinearVelocity,
        &ExtForce,
    )>,
    params: Res<PlayerParams>,
    mut gizmos: Gizmos<PhysicsGizmos>,
    dt: Res<Time<Substeps>>,
) {
    for (mut ground_cast, mut input, mut ground_force, LinearVelocity(velocity), ext_force) in
        query.iter_mut()
    {
        let Some(contact) = ground_cast.contact.as_ref() else {
            continue;
        };
        if ground_force.normal_force.dot(contact.normal.into()) < 1.0e-4 {
            continue;
        }
        let (target_vel, slope_force, target_force) = get_target_force(
            ext_force,
            &input,
            velocity,
            ground_force.normal_force,
            &params,
        );
        input.target_velocity = target_vel;
        gizmos.arrow(
            contact.contact_world,
            contact.contact_world + target_vel,
            RED,
        );
        gizmos.arrow(
            contact.contact_world,
            contact.contact_world + target_force,
            ORANGE,
        );

        ground_force.slope_force = slope_force;

        let target_force_dir = Dir3::new(target_force + ground_force.normal_force).unwrap();
        ground_cast.cast_dir = Dir3::new(
            ground_cast.cast_dir.as_vec3()
                + 12.0
                    * (-target_force_dir.as_vec3() - contact.contact_point.normalize_or_zero())
                    * dt.delta_secs(),
        )
        .unwrap();
    }
}

pub fn update_angular_spring(
    mut query: Query<(
        &GroundCast,
        &AngularVelocity,
        &mut GroundForce,
        &Rotation,
        &PlayerInput,
        &ExtForce,
    )>,
    params: Res<PlayerParams>,
) {
    for (
        ground_cast,
        AngularVelocity(angular_velocity),
        mut ground_force,
        Rotation(quat),
        input,
        ext_force,
    ) in query.iter_mut()
    {
        let Some(contact) = ground_cast.contact.as_ref() else {
            continue;
        };
        let mut target_up = Dir3::new(
            -ext_force
                .0
                .normalize_or_zero()
                .lerp((contact.contact_point.normalize()).into(), 0.8),
        )
        .unwrap_or(Dir3::Y);
        if input.jumping {
            target_up = Dir3::Y;
        }
        let target_right = Dir3::new(input.target_velocity.cross(target_up.into()))
            .unwrap_or(Dir3::new((*quat * Vec3::NEG_Z).cross(target_up.into())).unwrap());
        let angular_spring = params.springs.get_angular_spring(false);

        let apply_turn = if ground_force.slipping { 0.0 } else { 1.0 };

        let angular_spring_torque = angular_spring.stiffness
            * Quat::from_rotation_arc(*quat * Vec3::Y, target_up.into()).to_scaled_axis()
            + apply_turn
                * angular_spring.turn_stiffness
                * Quat::from_rotation_arc(*quat * Vec3::X, target_right.into()).to_scaled_axis()
            - angular_spring.damping * angular_velocity.clone();

        ground_force.angular_spring_torque = angular_spring_torque;
    }
}

pub fn aerial_spring(
    mut query: Query<(
        &GroundCast,
        &AirPrediction,
        &Rotation,
        &AngularVelocity,
        &mut ExternalTorque,
    )>,
    params: Res<PlayerParams>,
) {
    for (ground_cast, air_prediction, Rotation(quat), AngularVelocity(angular_vel), mut torque) in
        query.iter_mut()
    {
        if let Some(contact) = air_prediction.predicted_contact.as_ref() {
            if ground_cast.contact.is_some() {
                continue;
            }
            let capsule_forward = *quat * Vec3::NEG_Z;
            let capsule_up = *quat * Vec3::Y;
            let capsule_right = *quat * Vec3::X;
            let target_up = -ground_cast.cast_dir;
            let target_forward = contact
                .contact_velocity
                .reject_from(contact.contact_normal.into());
            let target_right = target_forward
                .cross(target_up.into())
                .try_normalize()
                .unwrap_or(
                    capsule_forward
                        .cross(target_up.into())
                        .try_normalize()
                        .unwrap(),
                );
            let angular_spring = &params.springs.angular_spring_aerial;
            let angular_spring_torque = angular_spring.stiffness
                * Quat::from_rotation_arc(capsule_up, target_up.into()).to_scaled_axis()
                + (angular_spring.turn_stiffness
                    * Quat::from_rotation_arc(capsule_right, target_right).to_scaled_axis())
                - angular_spring.damping * angular_vel.clone();
            torque.clear();
            torque.apply_torque(angular_spring_torque);
        }
    }
}

pub fn set_forces(
    mut query: Query<(
        &mut ExternalForce,
        &mut ExternalTorque,
        &GroundCast,
        &mut GroundForce,
    )>,
    mut gizmos: Gizmos<PhysicsGizmos>,
) {
    for (mut force, mut torque, ground_cast, mut ground_force) in query.iter_mut() {
        if let Some(contact) = ground_cast.contact.as_ref() {
            let friction_force_limit = ground_force.normal_force.length() * GLOBAL_FRICTION;
            ground_force.angle_force = -contact.normal.cross(ground_force.angular_spring_torque)
                / (contact.normal.dot(contact.contact_point));
            ground_force.slipping = (ground_force.tangential_force + ground_force.angle_force)
                .length()
                > friction_force_limit;
            let total_tangential_force = (ground_force.tangential_force + ground_force.angle_force)
                .clamp_length_max(friction_force_limit);

            torque.clear();
            torque.apply_torque(
                ground_force.angular_spring_torque
                    - contact.contact_point.cross(ground_force.angle_force),
            );
            force.clear();
            force.apply_force_at_point(
                total_tangential_force + ground_force.normal_force,
                contact.contact_point,
                Vec3::ZERO,
            );

            gizmos.arrow(
                contact.contact_world,
                contact.contact_world + ground_force.tangential_force,
                GREEN,
            );
            gizmos.arrow(
                contact.contact_world,
                contact.contact_world + ground_force.normal_force,
                BLUE,
            );
            gizmos.arrow(
                contact.contact_world + total_tangential_force,
                contact.contact_world + total_tangential_force + ground_force.angle_force,
                VIOLET,
            );
            gizmos.arrow(
                contact.contact_world,
                contact.contact_world + total_tangential_force + ground_force.normal_force,
                ORANGE,
            );
            gizmos.arrow(
                contact.contact_world - contact.contact_point,
                contact.contact_world + ground_force.slope_force - contact.contact_point,
                CYAN_100,
            );
        }
    }
}

fn draw_debug_gizmos(
    mut query: Query<(
        &Position,
        &Rotation,
        &LinearVelocity,
        &GroundCast,
        &AirPrediction,
        &GroundForce,
    )>,
    mut physics_gizmos: Gizmos<PhysicsGizmos>,
) {
    for (
        Position(pos),
        Rotation(quat),
        LinearVelocity(vel),
        ground_spring,
        air_prediction,
        ground_force,
    ) in query.iter_mut()
    {
        let pos = pos.clone();
        if let Some(contact) = &air_prediction.predicted_contact {
            physics_gizmos.sphere(
                Isometry3d::from_translation(contact.contact_world),
                0.1,
                Color::from(RED),
            );
        }
        if let Some(ground_contact) = ground_spring.contact.as_ref() {
            let contact = pos + ground_contact.contact_point;
            let normal = ground_contact.normal;
            let contact_color = if ground_force.slipping {
                Color::from(RED)
            } else {
                Color::from(GREEN)
            };
            physics_gizmos.sphere(
                Isometry3d::from_translation(
                    pos.clone() + ground_contact.toi * ground_spring.cast_dir,
                ),
                CAST_RADIUS,
                contact_color,
            );
            let tangent_vel = vel.reject_from(normal.into());
            physics_gizmos.arrow(contact, contact + tangent_vel, Color::from(ORANGE));
        } else {
            // draw circle at end of cast
            physics_gizmos.sphere(
                Isometry3d::from_translation(
                    pos + (RigBone::max_contact_dist() - CAST_RADIUS) * ground_spring.cast_dir,
                ),
                CAST_RADIUS,
                Color::BLACK,
            );
        }
        physics_gizmos
            .primitive_3d(
                &Capsule3d::new(CAPSULE_RADIUS, CAPSULE_HEIGHT - CAPSULE_RADIUS * 2.0),
                Isometry3d::new(pos.clone(), quat.clone()),
                Color::WHITE,
            )
            .resolution(6);
        physics_gizmos.cross(
            Isometry3d {
                rotation: quat.clone(),
                translation: pos.into(),
            },
            0.1,
            Color::WHITE,
        );
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
    external_force: &ExtForce,
    input: &PlayerInput,
    velocity: &Vec3,
    normal_force: Vec3,
    params: &PlayerParams,
) -> (Vec3, Vec3, Vec3) {
    // assert!(normal_force.length() > 0.0);
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

    let mut target_force = 0.3 * (target_vel - tangent_vel);

    let friction_force_margin = normal_force.length() * GLOBAL_FRICTION * FRICTION_MARGIN;

    if (target_force - slope_force).length() > friction_force_margin {
        target_force = add_results_in_length(
            target_force.normalize_or_zero(),
            -slope_force,
            friction_force_margin,
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
            (
                evaluate_ground_cast,
                physics_input,
                evaluate_ground_spring,
                update_cast_dir,
                update_angular_spring,
                set_forces,
                aerial_spring,
                draw_debug_gizmos, // update_forces,
            )
                .chain()
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
