use bevy::{prelude::*, transform::TransformSystem};
use bevy_xpbd_3d::{math::PI, prelude::*, SubstepSchedule, SubstepSet};
use leafwing_input_manager::prelude::*;

const CAPSULE_RADIUS: f32 = 0.2;
const CAPSULE_HEIGHT: f32 = 4.0 * CAPSULE_RADIUS;

#[derive(Actionlike, PartialEq, Eq, Hash, Clone, Debug, Reflect)]
pub enum Action {
    Jump,
    Move,
    View,
    Respawn,
}

impl Action {
    fn default_input_map() -> InputMap<Self> {
        let mut input_map = InputMap::default();
        input_map.insert(Self::Move, VirtualDPad::wasd());
        input_map.insert(Self::Jump, KeyCode::Space);
        input_map.insert(Self::Respawn, KeyCode::KeyR);
        input_map.insert(Self::View, DualAxis::mouse_motion());
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
}

#[derive(Component, Reflect, Debug, Default)]
struct PhysicsDebugInfo {
    grounded: bool,
    spring_force: Vec3,
    spring_torque: Vec3,
    normal_force: Vec3,
    shape_toi: f32,
    spring_dir: Vec3,
    contact_point: Vec3,
    position: Vec3,
    torque_cm_force: Vec3,
    ground_normal: Vec3,
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
                CollisionLayers::new(Layer::Player, Layer::Platform),
                RigidBody::default(),
                Name::new("PlayerBody"),
                Position::from(transform.translation()),
                Transform::from_translation(transform.translation()),
                GlobalTransform::default(),
                Player,
                ExternalForce::default().with_persistence(false),
                ExternalTorque::default().with_persistence(false),
                ground_spring.clone(),
                angular_spring.clone(),
                InputManagerBundle::<Action>::with_map(Action::default_input_map()),
                PlayerMoveState::default(),
                PhysicsDebugInfo::default(),
            ))
            .id();
    }
}

#[derive(Component, Reflect, Debug)]
struct CameraAnchor;

fn spawn_camera(mut commands: Commands) {
    let camera_arm = 0.25 * Vec3::new(0.0, 8.0, 25.0);
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

fn player_controls(
    mut query: Query<(&ActionState<Action>, &mut PlayerMoveState), With<Player>>,
    mut q_cam: Query<(&CameraAnchor, &mut Transform)>,
) {
    for (action_state, mut move_state) in query.iter_mut() {
        let move_input = action_state
            .clamped_axis_pair(&Action::Move)
            .unwrap()
            .xy()
            .normalize_or_zero();
        let cam_input = action_state.axis_pair(&Action::View).unwrap().xy();
        if let Ok((_, mut cam_transform)) = q_cam.get_single_mut() {
            let mut euler_angles = cam_transform.rotation.to_euler(EulerRot::YXZ);
            euler_angles.0 += cam_input.x * -0.005;
            euler_angles.1 += cam_input.y * -0.005;
            cam_transform.rotation =
                Quat::from_euler(EulerRot::YXZ, euler_angles.0, euler_angles.1, 0.0);

            move_state.acc_dir = Quat::from_euler(EulerRot::YXZ, -euler_angles.0, 0.0, 0.0)
                * Vec3::new(move_input.x, 0.0, move_input.y);
            if action_state.pressed(&Action::Jump) {
                move_state.spring_height = CAPSULE_HEIGHT * 0.7;
            } else {
                move_state.spring_height = CAPSULE_HEIGHT * 1.3;
            }
        }
    }
}

fn respawn_player(
    mut commands: Commands,
    query: Query<Entity, (With<PlayerSpawn>, With<PlayerSpawned>)>,
    player: Query<Entity, With<Player>>,
    input_query: Query<&ActionState<Action>>,
) {
    for entity in query.iter() {
        for action_state in input_query.iter() {
            if action_state.just_pressed(&Action::Respawn) {
                if let Ok(player) = player.get_single() {
                    commands.entity(player).despawn_recursive();
                }
                commands.entity(entity).remove::<PlayerSpawned>();
            }
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
            /*
            gizmos.arrow(
                position.clone() + debug.contact_point,
                position.clone() + debug.contact_point + debug.ground_normal,
                Color::ORANGE,
            );*/
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
            &PlayerMoveState,
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
        move_state,
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
            CAPSULE_HEIGHT * 2.0,
            false,
            filter.clone(),
        ) {
            debug.grounded = true;
            debug.ground_normal = coll.normal1;

            let contact_point = coll.point2 + -from_up * coll.time_of_impact;
            debug.contact_point = contact_point;
            debug.shape_toi = coll.time_of_impact;
            let spring_vel = velocity.dot(coll.normal1) / (from_up.dot(coll.normal1));
            // println!("Time {:?}", coll.time_of_impact);
            let spring_force = (-spring.stiffness
                * (coll.time_of_impact - move_state.spring_height)
                - spring.damping * spring_vel)
                .max(0.0)
                * from_up;
            debug.spring_force = spring_force;
            let normal_force = spring_force.dot(coll.normal1) * coll.normal1;
            debug.normal_force = normal_force;
            let tangential_force = spring_force - normal_force;
            force.clear();
            force.apply_force_at_point(spring_force, 0.0 * contact_point, Vec3::ZERO);
            // println!("Force {:?}", force.force());
            let yaw = move_state.acc_dir.z.atan2(move_state.acc_dir.x);
            let pitch = -0.2 * PI * move_state.acc_dir.length();
            let target_quat = Quat::from_euler(EulerRot::YZX, yaw, pitch, 0.0);
            let target_up = target_quat * Vec3::Y;
            let delta_angle = from_up.angle_between(target_up);
            let delta_axis = from_up.cross(target_up).normalize_or_zero();
            let spring_torque = angular_spring.stiffness * delta_axis * delta_angle
                - (angular_spring.damping * angular_vel.clone());
            gizmos.arrow(
                position.clone(),
                position.clone() + angular_vel.clone(),
                Color::CYAN,
            );
            debug.spring_torque = spring_torque;
            let normal = coll.normal1;

            let cm_force = normal.cross(spring_torque) / (normal.dot(contact_point));

            debug.torque_cm_force = cm_force;
            force.apply_force(cm_force);
            torque.set_torque(spring_torque);
        }
    }
}

pub struct PlayerPlugin;

impl Plugin for PlayerPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(InputManagerPlugin::<Action>::default());
        app.register_type::<PlayerSpawn>();
        app.register_type::<DistanceJoint>();
        app.register_type::<PlayerGroundSpring>();
        app.register_type::<PlayerAngularSpring>();
        app.insert_resource(SubstepCount(12));
        app.insert_resource(PlayerGroundSpring {
            rest_length: 0.0,
            stiffness: 11.0,
            damping: 1.0,
        });
        app.insert_resource(PlayerAngularSpring {
            stiffness: 10.0,
            damping: 2.0,
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
