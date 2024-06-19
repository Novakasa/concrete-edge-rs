use bevy::{
    prelude::*,
    transform::{commands, TransformSystem},
};
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

fn player_controls(mut query: Query<(&ActionState<Action>, &mut PlayerMoveState), With<Player>>) {
    for (action_state, mut move_state) in query.iter_mut() {
        let dir = action_state
            .clamped_axis_pair(&Action::Move)
            .unwrap()
            .xy()
            .normalize_or_zero();
        println!("Move: {:?}", dir);
        move_state.acc_dir = Vec3::new(dir.x, 0.0, dir.y);
        if action_state.pressed(&Action::Jump) {
            move_state.spring_height = CAPSULE_HEIGHT * 0.7;
        } else {
            move_state.spring_height = CAPSULE_HEIGHT * 1.3;
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
    ) in query.iter_mut()
    {
        let from_up = *quat * Vec3::Y;
        if let Some(coll) = shape_cast.cast_shape(
            &Collider::sphere(CAPSULE_RADIUS),
            position.clone(),
            Quat::default(),
            Direction3d::new_unchecked(-from_up),
            CAPSULE_HEIGHT * 2.0,
            false,
            SpatialQueryFilter::from_mask(Layer::Platform),
        ) {
            gizmos.sphere(
                position.clone() + -from_up * coll.time_of_impact,
                Quat::IDENTITY,
                CAPSULE_RADIUS,
                Color::RED,
            );
            let contact_point = coll.point2 + -from_up * coll.time_of_impact;
            // println!("Time {:?}", coll.time_of_impact);
            let spring_force = (-spring.stiffness
                * (coll.time_of_impact - move_state.spring_height)
                - spring.damping * velocity.dot(from_up))
            .max(0.0);
            force.clear();
            force.apply_force_at_point(spring_force * from_up, contact_point, Vec3::ZERO);
            // println!("Force {:?}", force.force());
            let yaw = move_state.acc_dir.z.atan2(move_state.acc_dir.x);
            let pitch = move_state.acc_dir.length();
            let target_quat = Quat::from_euler(EulerRot::YZX, yaw, -0.2 * PI * pitch, 0.0);
            let target_up = target_quat * Vec3::Y;
            let delta_angle = from_up.angle_between(target_up);
            let delta_axis = from_up.cross(target_up).normalize_or_zero();
            //println!("From up dir: {:?}, Target up dir: {:?}", from_up, target_up);
            // println!("Delta angle: {:?}", delta_angle);
            let spring_torque = angular_spring.stiffness * delta_axis * delta_angle
                - (angular_spring.damping * angular_vel.clone())
                - force.torque();
            let cm_force = -(spring_torque + force.torque()).cross(contact_point);
            if false {
                if cm_force.dot(target_up.cross(from_up)) > 0.0 && false {
                    println!("accelerating");
                    force.apply_force(cm_force);
                } else {
                    println!("decelerating");
                    force.apply_force(cm_force);
                }
            }
            torque.set_torque(spring_torque);
            let acc_dir = Vec3::Y.cross(spring_torque).normalize_or_zero();
            let dir_cross_contact = acc_dir.cross(contact_point);
            if dir_cross_contact.length() > 0.00001 {
                let f_acc = acc_dir * (spring_force * coll.normal1.cross(contact_point)).length()
                    / (dir_cross_contact.length());
                // force.apply_force(f_acc);
            }
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
