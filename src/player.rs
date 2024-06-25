use bevy::{core_pipeline::core_2d::graph::input, prelude::*, transform::TransformSystem};
use bevy_xpbd_3d::{math::PI, prelude::*, SubstepSchedule, SubstepSet};
use leafwing_input_manager::prelude::*;

const CAPSULE_RADIUS: f32 = 0.2;
const CAPSULE_HEIGHT: f32 = 4.0 * CAPSULE_RADIUS;

#[derive(Actionlike, PartialEq, Eq, Hash, Clone, Debug, Reflect)]
pub enum PlayerAction {
    Jump,
    Move,
    View,
    Respawn,
    Menu,
}

impl PlayerAction {
    fn default_input_map() -> InputMap<Self> {
        let mut input_map = InputMap::default();
        input_map.insert(Self::Move, VirtualDPad::wasd());
        input_map.insert(Self::Jump, KeyCode::Space);
        input_map.insert(Self::Respawn, KeyCode::KeyR);
        input_map.insert(Self::View, DualAxis::mouse_motion());
        input_map.insert(Self::Menu, KeyCode::Escape);
        input_map
    }
}

#[derive(PhysicsLayer)]
pub enum Layer {
    Player,
    Platform,
}

#[derive(Component, Reflect, Debug)]
#[reflect(Component)]
pub struct PlayerSpawn {
    test: i32,
}

#[derive(Component, Debug)]
struct PlayerSpawned;

#[derive(Component, Reflect, Debug)]
struct PlayerFeet;

#[derive(Component, Reflect, Debug)]
struct Player;

#[derive(Component, Reflect, Debug, Resource, Clone, Default)]
#[reflect(Component, Resource)]
struct PlayerGroundSpring {
    rest_length: f32,
    stiffness: f32,
    damping: f32,
}

impl PlayerGroundSpring {
    fn force(&self, length: f32, vel: f32) -> f32 {
        (-self.stiffness * (length - self.rest_length) - self.damping * vel).max(0.0)
    }
}

#[derive(Component, Reflect, Debug, Resource, Clone, Default)]
#[reflect(Component, Resource)]
struct PlayerAngularSpring {
    stiffness: f32,
    damping: f32,
}

#[derive(Component, Reflect, Debug, Default)]
struct PlayerMoveState {
    acc_dir: Vec3,
    spring_height: f32,
    prev_vel: Vec3,
}

#[derive(Component, Reflect, Debug, Default)]
struct PhysicsDebugInfo {
    grounded: bool,
    spring_force: Vec3,
    spring_torque: Vec3,
    normal_force: Vec3,
    tangential_force: Vec3,
    shape_toi: f32,
    spring_dir: Vec3,
    contact_point: Vec3,
    position: Vec3,
    torque_cm_force: Vec3,
    ground_normal: Vec3,
    tangent_vel: Vec3,
    target_vel: Vec3,
    target_force: Vec3,
}

fn spawn_player(
    mut commands: Commands,
    query: Query<(Entity, &GlobalTransform), (With<PlayerSpawn>, Without<PlayerSpawned>)>,
    ground_spring: Res<PlayerGroundSpring>,
    angular_spring: Res<PlayerAngularSpring>,
) {
    for (entity, transform) in query.iter() {
        commands.entity(entity).insert(PlayerSpawned);
        println!("Spawning player: {:?}", entity);
        let body = commands
            .spawn((
                Collider::capsule(CAPSULE_RADIUS * 2.0, CAPSULE_RADIUS),
                ColliderDensity(1.5),
                CollisionLayers::new(Layer::Player, Layer::Platform),
                RigidBody::default(),
                // Name::new("PlayerBody"),
                Position::from(transform.translation()),
                Transform::from_translation(transform.translation()),
                GlobalTransform::default(),
                Player,
                ExternalForce::default().with_persistence(false),
                ExternalTorque::default().with_persistence(false),
                ground_spring.clone(),
                angular_spring.clone(),
                InputManagerBundle::<PlayerAction>::with_map(PlayerAction::default_input_map()),
                PlayerMoveState::default(),
                PhysicsDebugInfo::default(),
            ))
            .id();
    }
}

#[derive(Component, Reflect, Debug)]
struct CameraAnchor;

fn spawn_camera(mut commands: Commands) {
    let camera_arm = 0.15 * Vec3::new(0.0, 8.0, 25.0);
    let transform =
        Transform::from_translation(camera_arm).looking_to(-camera_arm.normalize(), Vec3::Y);
    commands
        .spawn((
            TransformBundle::default(),
            Name::new("CameraAnchor"),
            CameraAnchor,
        ))
        .with_children(|builder| {
            builder
                .spawn(Camera3dBundle {
                    projection: Projection::Perspective(PerspectiveProjection {
                        fov: PI / 3.0,
                        ..Default::default()
                    }),
                    transform: transform,
                    ..Default::default()
                })
                .insert(Name::new("PlayerCamera"));
        });
}

fn track_camera(
    query: Query<(&GlobalTransform), With<Player>>,
    mut camera_query: Query<&mut Transform, With<CameraAnchor>>,
) {
    for (player_transform) in query.iter() {
        for mut transform in camera_query.iter_mut() {
            transform.translation = player_transform.translation().clone();
        }
    }
}

fn respawn_player(
    mut commands: Commands,
    query: Query<Entity, (With<PlayerSpawn>, With<PlayerSpawned>)>,
    player: Query<Entity, With<Player>>,
    input_query: Query<&ActionState<PlayerAction>>,
) {
    for entity in query.iter() {
        for action_state in input_query.iter() {
            if action_state.just_pressed(&PlayerAction::Respawn) {
                if let Ok(player) = player.get_single() {
                    commands.entity(player).despawn_recursive();
                }
                commands.entity(entity).remove::<PlayerSpawned>();
            }
        }
    }
}

fn player_controls(
    mut query: Query<
        (
            &ActionState<PlayerAction>,
            &mut PlayerMoveState,
            &mut PlayerGroundSpring,
            &mut PlayerAngularSpring,
        ),
        With<Player>,
    >,
    mut q_cam: Query<(&CameraAnchor, &mut Transform)>,
) {
    for (action_state, mut move_state, mut spring, mut angular_spring) in query.iter_mut() {
        let move_input = action_state
            .clamped_axis_pair(&PlayerAction::Move)
            .unwrap()
            .xy()
            .normalize_or_zero();
        // println!("{:?}", move_input);
        let cam_input = action_state.axis_pair(&PlayerAction::View).unwrap().xy();
        if let Ok((_, mut cam_transform)) = q_cam.get_single_mut() {
            let mut euler_angles = cam_transform.rotation.to_euler(EulerRot::YXZ);
            euler_angles.0 += cam_input.x * -0.005;
            euler_angles.1 += cam_input.y * -0.005;
            cam_transform.rotation =
                Quat::from_euler(EulerRot::YXZ, euler_angles.0, euler_angles.1, 0.0);

            move_state.acc_dir = Quat::from_euler(EulerRot::YXZ, euler_angles.0, 0.0, 0.0)
                * Vec3::new(move_input.x, 0.0, -move_input.y);
            // println!("{:?}", move_state.acc_dir);
            if action_state.pressed(&PlayerAction::Jump) {
                spring.rest_length = CAPSULE_HEIGHT * 2.0;
                spring.damping = 2.0;
                angular_spring.stiffness = 0.0;
            } else {
                spring.rest_length = CAPSULE_HEIGHT * 1.0;
                spring.damping = 2.0;
                angular_spring.stiffness = 15.0;
            }
        }
    }
}

fn update_ground_force(
    mut query: Query<
        (
            &mut ExternalForce,
            &mut ExternalTorque,
            &Position,
            &Rotation,
            &LinearVelocity,
            &AngularVelocity,
            &PlayerGroundSpring,
            &PlayerAngularSpring,
            &mut PlayerMoveState,
            &mut PhysicsDebugInfo,
        ),
        With<Player>,
    >,
    shape_cast: SpatialQuery,
    mut gizmos: Gizmos,
    dt: Res<Time<Substeps>>,
) {
    for (
        mut force,
        mut torque,
        Position(position),
        Rotation(quat),
        LinearVelocity(velocity),
        AngularVelocity(angular_vel),
        spring,
        angular_spring,
        mut move_state,
        mut debug,
    ) in query.iter_mut()
    {
        let filter = SpatialQueryFilter::from_mask(Layer::Platform);
        let from_up = *quat * Vec3::Y;
        if let Some(coll) = shape_cast.cast_shape(
            &Collider::sphere(CAPSULE_RADIUS),
            position.clone(),
            Quat::IDENTITY,
            Direction3d::new_unchecked(-from_up.normalize_or_zero()),
            CAPSULE_HEIGHT * 1.5,
            false,
            filter.clone(),
        ) {
            let normal = coll.normal1;
            debug.grounded = true;
            debug.ground_normal = normal;

            let contact_point = coll.point2 + -from_up * coll.time_of_impact;
            debug.contact_point = contact_point;
            debug.shape_toi = coll.time_of_impact;
            let spring_vel = velocity.dot(normal) / (from_up.dot(normal));
            // println!("Time {:?}", coll.time_of_impact);
            let spring_force = spring.force(coll.time_of_impact, spring_vel) * from_up;
            debug.spring_force = spring_force;
            let normal_force = spring_force.dot(normal) * normal;
            debug.normal_force = normal_force;
            let tangential_force = spring_force - normal_force;
            debug.tangential_force = tangential_force;

            let tangent_plane = normal.cross(Vec3::Y).normalize_or_zero();
            let tangent_slope = normal.cross(tangent_plane).normalize_or_zero();
            let tangent_z = Vec3::X.cross(normal).normalize_or_zero();
            let tangent_x = -tangent_z.cross(normal);
            let acc_tangent = move_state.acc_dir.x * tangent_x + move_state.acc_dir.z * tangent_z;
            let tangent_vel = *velocity - velocity.dot(normal) * normal;
            let prev_tangent_vel = move_state.prev_vel - move_state.prev_vel.dot(normal) * normal;

            debug.tangent_vel = tangent_vel;

            let target_vel = acc_tangent * 7.0;
            debug.target_vel = target_vel;
            let denominator = 1.0 - tangent_slope.dot(Vec3::Y).powi(2);
            let slope_force = if denominator == 0.0 {
                Vec3::ZERO
            } else {
                -tangent_slope.dot(Vec3::Y) * normal_force.dot(Vec3::Y) * tangent_slope
                    / denominator
            };

            let max_lean = 0.25 * PI;
            let max_force = max_lean.tan() * normal_force.length();

            let mut target_force = 0.2 * (target_vel - tangent_vel);
            if max_force > 0.0 {
                target_force = (target_force.length() / max_force).powi(1)
                    * max_force
                    * target_force.normalize_or_zero()
            }
            target_force -= 0.0000 * (tangent_vel - prev_tangent_vel) / dt.delta_seconds();

            if target_force.length() > max_force {
                target_force =
                    add_results_in_length(target_force.normalize_or_zero(), -slope_force, max_force)
                        .unwrap_or(target_force)
            }
            target_force -= slope_force;
            move_state.prev_vel = velocity.clone();
            debug.target_force = target_force;
            // println!("{:?}, {:?}", target_force, normal_force);

            force.clear();
            force.apply_force_at_point(spring_force, 0.0 * contact_point, Vec3::ZERO);

            let pitch = (target_force.length() / normal_force.length())
                .atan()
                .min(max_lean);

            let yaw = target_force
                .dot(tangent_x)
                .atan2(target_force.dot(tangent_z));

            let target_quat = Quat::from_rotation_arc(Vec3::Y, normal)
                * Quat::from_euler(EulerRot::YXZ, yaw, pitch, 0.0);
            let target_up = target_quat * ((Vec3::Y).normalize_or_zero());
            let delta_angle = from_up.angle_between(target_up);
            let delta_axis = from_up.cross(target_up).normalize_or_zero();
            let spring_torque = angular_spring.stiffness * delta_axis * delta_angle
                - (angular_spring.damping * angular_vel.clone());
            debug.spring_torque = spring_torque;

            let y_damping = angular_vel.y * -0.1;
            torque.apply_torque(y_damping * Vec3::Y);

            let cm_force = normal.cross(spring_torque) / (normal.dot(contact_point));

            debug.torque_cm_force = cm_force;
            // force.apply_force_at_point(contact_point, cm_force, Vec3::ZERO);
            force.apply_force(cm_force);
            torque.set_torque(spring_torque);
        } else {
            debug.grounded = false;
            force.clear();
            torque.clear();
        }
    }
}

fn draw_debug_gizmos(
    query: Query<(&PhysicsDebugInfo, &Position, &Rotation), With<Player>>,
    mut gizmos: Gizmos,
) {
    for (debug, Position(position), Rotation(quat)) in query.iter() {
        if debug.grounded {
            gizmos.sphere(debug.contact_point, Quat::IDENTITY, 0.1, Color::RED);
            gizmos.arrow(
                position.clone() + debug.contact_point,
                position.clone() + debug.contact_point + debug.spring_force,
                Color::GREEN,
            );
            gizmos.arrow(
                position.clone() + debug.contact_point,
                position.clone() + debug.contact_point + debug.normal_force,
                Color::BLUE,
            );

            gizmos.arrow(
                position.clone() + debug.contact_point,
                position.clone() + debug.contact_point + debug.tangential_force,
                Color::CYAN,
            );

            gizmos.arrow(
                position.clone() + debug.contact_point,
                position.clone() + debug.contact_point + debug.tangent_vel,
                Color::ORANGE,
            );
            gizmos.arrow(
                position.clone() + debug.contact_point,
                position.clone() + debug.contact_point + debug.target_vel,
                Color::GREEN,
            );
            gizmos.arrow(
                position.clone() + debug.contact_point,
                position.clone() + debug.contact_point + debug.target_force,
                Color::RED,
            );
            gizmos.arrow(
                position.clone(),
                position.clone() + debug.torque_cm_force,
                Color::YELLOW,
            );
            gizmos.arrow(
                position.clone(),
                position.clone() + debug.spring_torque,
                Color::CYAN,
            );
            gizmos.sphere(
                position.clone() + -debug.spring_force.normalize_or_zero() * debug.shape_toi,
                Quat::IDENTITY,
                CAPSULE_RADIUS,
                Color::RED,
            );
        }
        gizmos
            .primitive_3d(
                Capsule3d::new(CAPSULE_RADIUS, CAPSULE_HEIGHT - CAPSULE_RADIUS * 2.0),
                position.clone(),
                quat.clone(),
                Color::WHITE,
            )
            .segments(6);
    }
}

fn add_results_in_length(dir: Vec3, rhs: Vec3, combined_length: f32) -> Option<Vec3> {
    let dot = dir.dot(rhs);
    let discriminant = dot.powi(2) - rhs.dot(rhs) + combined_length.powi(2);
    if discriminant < 0.0 {
        // godot_print!("Discriminant is negative");
        return None;
    }
    Some((-dot + discriminant.sqrt()) * dir)
}

pub struct PlayerPlugin;

impl Plugin for PlayerPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(InputManagerPlugin::<PlayerAction>::default());
        app.register_type::<PlayerSpawn>();
        app.register_type::<DistanceJoint>();
        app.register_type::<PlayerGroundSpring>();
        app.register_type::<PlayerAngularSpring>();
        app.insert_resource(SubstepCount(12));
        app.insert_resource(PlayerGroundSpring {
            rest_length: 0.0,
            stiffness: 15.0,
            damping: 2.0,
        });
        app.insert_resource(PlayerAngularSpring {
            stiffness: 15.0,
            damping: 3.0,
        });
        app.add_systems(Startup, spawn_camera);
        app.add_systems(
            Update,
            (
                spawn_player,
                player_controls,
                respawn_player,
                draw_debug_gizmos.after(PhysicsSet::Sync),
                track_camera
                    .after(PhysicsSet::Sync)
                    .before(TransformSystem::TransformPropagate),
            ),
        );
        app.add_systems(
            SubstepSchedule,
            update_ground_force.before(SubstepSet::Integrate),
        );
    }
}
