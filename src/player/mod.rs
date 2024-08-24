use std::f32::consts::PI;

use avian3d::prelude::*;
use bevy::{
    color::palettes::{
        css::{BLUE, GRAY, GREEN, ORANGE, PINK, RED, YELLOW},
        tailwind::CYAN_100,
    },
    ecs::system::SystemParam,
    prelude::*,
};
use dynamics::integrator::IntegrationSet;
use leafwing_input_manager::prelude::*;
use physics::{
    PhysicsDebugInfo, PlayerAngularSpring, PlayerGroundSpring, PlayerMoveState, CAPSULE_HEIGHT,
    CAPSULE_RADIUS, CAST_RADIUS,
};

mod physics;

#[derive(States, Debug, Clone, PartialEq, Eq, Hash)]
pub enum DebugState {
    None,
    Colliders,
    Torque,
    Forces,
}

#[derive(PartialEq, Eq, Hash, Clone, Debug, Reflect)]
pub enum PlayerAction {
    Jump,
    Move,
    View,
    Respawn,
    Menu,
    ViewMode,
}

impl Actionlike for PlayerAction {
    fn input_control_kind(&self) -> InputControlKind {
        match self {
            Self::Move | Self::View => InputControlKind::DualAxis,
            _ => InputControlKind::Button,
        }
    }
}

impl PlayerAction {
    fn default_input_map() -> InputMap<Self> {
        let mut input_map = InputMap::default();
        input_map.insert_dual_axis(Self::Move, KeyboardVirtualDPad::WASD);
        input_map.insert(Self::Jump, KeyCode::Space);
        input_map.insert(Self::Respawn, KeyCode::KeyR);
        input_map.insert_dual_axis(Self::View, MouseMove::default());
        input_map.insert(Self::Menu, KeyCode::Escape);
        input_map.insert(Self::ViewMode, KeyCode::Tab);
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
pub struct Player;

#[derive(Debug, Default)]
struct FootTravelInfo {
    time: f32,
    pos: Vec3,
    pos0: Vec3,
    duration: f32,
    target: Option<Vec3>,
}

impl FootTravelInfo {
    fn unlocked(pos: Vec3, duration: f32) -> Self {
        Self {
            time: 0.0,
            pos,
            pos0: pos,
            duration,
            target: None,
        }
    }
}

#[derive(Debug)]
enum FootState {
    Locked(Vec3),
    Unlocked(FootTravelInfo),
}

impl FootState {
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
struct ProceduralRigState {
    hip_pos: Vec3,
    foot_states: [FootState; 2],
}

fn spawn_player(
    mut commands: Commands,
    query: Query<(Entity, &GlobalTransform), (With<PlayerSpawn>, Without<PlayerSpawned>)>,
    ground_spring: Res<physics::PlayerGroundSpring>,
    angular_spring: Res<physics::PlayerAngularSpring>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    debug_state: Res<State<DebugState>>,
) {
    for (entity, transform) in query.iter() {
        commands.entity(entity).insert(PlayerSpawned);
        println!("Spawning player: {:?}", entity);
        let capsule = meshes.add(Mesh::from(Capsule3d::new(
            physics::CAPSULE_RADIUS,
            physics::CAPSULE_HEIGHT - 2.0 * physics::CAPSULE_RADIUS,
        )));
        let material = materials.add(Color::WHITE);
        let visibility = if debug_state.get() == &DebugState::None {
            Visibility::Visible
        } else {
            Visibility::Hidden
        };
        let _body = commands
            .spawn((
                Collider::capsule(
                    physics::CAPSULE_RADIUS,
                    physics::CAPSULE_HEIGHT - 2.0 * physics::CAPSULE_RADIUS,
                ),
                ColliderDensity(1.5),
                CollisionLayers::new(Layer::Player, Layer::Platform),
                RigidBody::default(),
                Position::from(transform.translation()),
                Transform::from_translation(transform.translation()),
                GlobalTransform::default(),
                Player,
                ExternalForce::default().with_persistence(false),
                ExternalTorque::default().with_persistence(false),
                ground_spring.clone(),
                angular_spring.clone(),
                InputManagerBundle::<PlayerAction>::with_map(PlayerAction::default_input_map()),
                physics::PlayerMoveState::new(),
                physics::PhysicsDebugInfo::default(),
            ))
            .insert(MaterialMeshBundle {
                mesh: capsule,
                material,
                visibility,
                ..Default::default()
            })
            .insert((Restitution::new(0.0), Friction::new(0.0)))
            .insert((ProceduralRigState::default(), Name::new("PlayerBody")))
            .id();
    }
}

#[derive(Component, Reflect, Debug, Default)]
struct CameraAnchor3rdPerson {
    yaw: f32,
    pitch: f32,
}

#[derive(Component, Reflect, Debug, Default)]
struct CameraAnchor1stPerson {
    yaw: f32,
    pitch: f32,
}

fn spawn_camera_3rd_person(mut commands: Commands) {
    let camera_arm = 0.15 * Vec3::new(0.0, 8.0, 25.0);
    let transform =
        Transform::from_translation(camera_arm).looking_to(-camera_arm.normalize(), Vec3::Y);
    commands
        .spawn((
            TransformBundle::default(),
            Name::new("CameraAnchor"),
            CameraAnchor3rdPerson::default(),
        ))
        .with_children(|builder| {
            builder
                .spawn(Camera3dBundle {
                    projection: Projection::Perspective(PerspectiveProjection {
                        fov: PI / 3.0,
                        ..Default::default()
                    }),
                    camera: Camera {
                        is_active: true,
                        ..Default::default()
                    },
                    transform: transform,
                    ..Default::default()
                })
                .insert(Name::new("PlayerCamera"));
        });
}

fn spawn_camera_1st_person(mut commands: Commands) {
    let camera_arm = Vec3::new(0.0, 0.2 * physics::CAPSULE_HEIGHT, 0.0);
    let transform = Transform::from_translation(camera_arm);
    commands
        .spawn((
            TransformBundle::default(),
            Name::new("CameraAnchor"),
            CameraAnchor1stPerson::default(),
        ))
        .with_children(|builder| {
            builder
                .spawn(Camera3dBundle {
                    projection: Projection::Perspective(PerspectiveProjection {
                        fov: PI / 3.0,
                        ..Default::default()
                    }),
                    camera: Camera {
                        is_active: false,
                        ..Default::default()
                    },
                    transform: transform,
                    ..Default::default()
                })
                .insert(Name::new("PlayerCamera"));
        });
}

fn toggle_active_view(
    anchor1: Query<(&CameraAnchor1stPerson, &Children)>,
    anchor3: Query<(&CameraAnchor3rdPerson, &Children)>,
    mut cams: Query<&mut Camera>,
    input_query: Query<&ActionState<PlayerAction>>,
) {
    for input in input_query.iter() {
        if input.just_pressed(&PlayerAction::ViewMode) {
            let mut cam1 = cams
                .get_mut(*anchor1.get_single().unwrap().1.iter().next().unwrap())
                .unwrap();
            cam1.is_active = !cam1.is_active;
            let mut cam3 = cams
                .get_mut(*anchor3.get_single().unwrap().1.iter().next().unwrap())
                .unwrap();
            cam3.is_active = !cam3.is_active;
        }
    }
}

fn track_camera_3rd_person(
    query: Query<&Position, With<Player>>,
    mut camera_query: Query<(&mut Transform, &CameraAnchor3rdPerson)>,
) {
    for Position(pos) in query.iter() {
        for (mut transform, anchor3) in camera_query.iter_mut() {
            transform.translation = pos.clone();
            transform.rotation = Quat::from_euler(EulerRot::YXZ, anchor3.yaw, anchor3.pitch, 0.0);
        }
    }
}

fn track_camera_1st_person(
    query: Query<(&Position, &Rotation), With<Player>>,
    mut camera_query: Query<(&mut Transform, &CameraAnchor1stPerson)>,
) {
    for (Position(pos), Rotation(quat)) in query.iter() {
        let up_dir = *quat * Vec3::Y;
        let pos = pos.clone() + up_dir * physics::CAPSULE_HEIGHT * 0.3;
        for (mut transform, cam1) in camera_query.iter_mut() {
            let view_unrolled = Quat::from_euler(EulerRot::YXZ, cam1.yaw, cam1.pitch, 0.0);
            let forward = view_unrolled * Vec3::NEG_Z;
            *transform = transform.with_translation(pos).looking_to(forward, up_dir);
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
            &mut physics::PlayerMoveState,
            &mut physics::PlayerGroundSpring,
            &mut physics::PlayerAngularSpring,
        ),
        With<Player>,
    >,
    mut q_cam3: Query<&mut CameraAnchor3rdPerson, Without<CameraAnchor1stPerson>>,
    mut q_cam1: Query<&mut CameraAnchor1stPerson, Without<CameraAnchor3rdPerson>>,
) {
    for (action_state, mut move_state, mut spring, mut angular_spring) in query.iter_mut() {
        let move_input = action_state
            .clamped_axis_pair(&PlayerAction::Move)
            .normalize_or_zero();
        // println!("{:?}", move_input);
        let cam_input = action_state.axis_pair(&PlayerAction::View);
        if let Ok(mut cam3) = q_cam3.get_single_mut() {
            cam3.yaw += cam_input.x * -0.005;
            cam3.pitch = (cam3.pitch + cam_input.y * -0.005).clamp(-0.5 * PI, 0.5 * PI);

            if let Ok(mut cam1) = q_cam1.get_single_mut() {
                cam1.yaw = cam3.yaw;
                cam1.pitch = cam3.pitch;
            }

            move_state.acc_dir = Quat::from_euler(EulerRot::YXZ, cam3.yaw, 0.0, 0.0)
                * Vec3::new(move_input.x, 0.0, -move_input.y);
            // println!("{:?}", move_state.acc_dir);
            if action_state.pressed(&PlayerAction::Jump) {
                spring.rest_length = CAPSULE_HEIGHT * 1.4 + CAPSULE_RADIUS;
                spring.min_damping = 1.0;
                spring.stiffness = 15.0;
                angular_spring.stiffness = 0.3;
            } else {
                spring.rest_length = CAPSULE_HEIGHT * 0.7 + CAPSULE_RADIUS;
                spring.min_damping = 1.5;
                spring.stiffness = 15.0;
                angular_spring.stiffness = 0.5;
                angular_spring.damping = 0.2;
                angular_spring.turn_stiffness = 0.4;
            }
        }
    }
}

#[derive(SystemParam)]
struct ProceduralRig<'w, 's> {
    query: Query<
        'w,
        's,
        (
            &'static Position,
            &'static Rotation,
            &'static mut ProceduralRigState,
            &'static PlayerMoveState,
            &'static Mass,
            &'static LinearVelocity,
        ),
        With<Player>,
    >,
}

impl<'w, 's> ProceduralRig<'w, 's> {}

fn update_procedural_steps(
    mut query: Query<
        (
            &Position,
            &Rotation,
            &mut ProceduralRigState,
            &PlayerMoveState,
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

            let (i_unlock, i_lock) = match (&rig_state.foot_states[0], &rig_state.foot_states[1]) {
                (FootState::Locked(pos0), FootState::Locked(pos1)) => {
                    if (window_pos_ahead - *pos0).length() > (window_pos_ahead - *pos1).length() {
                        (0, 1)
                    } else {
                        (1, 0)
                    }
                }
                (FootState::Locked(_), FootState::Unlocked(_)) => (0, 1),
                (FootState::Unlocked(_), FootState::Locked(_)) => (1, 0),
                (FootState::Unlocked(info0), FootState::Unlocked(info1)) => {
                    match (info0.target, info1.target) {
                        (Some(_), None) => (1, 0),
                        (None, Some(_)) => (0, 1),
                        (Some(target0), Some(target1)) => {
                            if (info0.pos - target0).length() > (info1.pos - target1).length() {
                                (0, 1)
                            } else {
                                (1, 0)
                            }
                        }
                        _ => (0, 1),
                    }
                }
            };

            let offset_length = 0.3 * CAPSULE_RADIUS;

            for (i, state) in rig_state.foot_states.iter_mut().enumerate() {
                let lr = if i == 0 { -1.0 } else { 1.0 };
                let foot_offset = right_tangent * lr * offset_length;
                match state {
                    FootState::Locked(pos) => {
                        *pos += slip_vel * dt;
                        let next_lock_pos = contact
                            + (acceleration * (travel_duration + 0.5 * lock_duration)
                                + tangential_vel)
                                * (travel_duration + 0.5 * lock_duration)
                            + foot_offset;
                        let _local_travel_dist = (window_pos_ahead - *pos).length();
                        let pos_to_next = (*pos - next_lock_pos).length();
                        let pos_to_contact = (*pos - contact).length();
                        if pos_to_next > min_step_size
                            && pos_to_contact > window_travel_dist * 0.5
                            && pos_to_contact > 1.3 * ahead_to_contact
                        {
                            *state = FootState::Unlocked(FootTravelInfo::unlocked(
                                *pos,
                                travel_duration,
                            ));
                        }
                    }
                    FootState::Unlocked(info) => {
                        if i != i_lock {
                            info.duration += dt;
                        }
                        info.time += dt;
                        if info.time > info.duration {
                            *state = FootState::Locked(window_pos_ahead + foot_offset);
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

fn set_visible<const VAL: bool>(mut query: Query<&mut Visibility, With<Player>>) {
    for mut visibility in query.iter_mut() {
        if VAL {
            *visibility = Visibility::Visible;
        } else {
            *visibility = Visibility::Hidden;
        }
    }
}

fn draw_debug_gizmos(
    mut query: Query<
        (
            &PhysicsDebugInfo,
            &Position,
            &Rotation,
            &PlayerMoveState,
            &ProceduralRigState,
        ),
        With<Player>,
    >,
    mut gizmos: Gizmos,
    debug_state: Res<State<DebugState>>,
) {
    for (debug, Position(position), Rotation(quat), move_state, steps) in query.iter_mut() {
        if debug.grounded {
            for (i, state) in steps.foot_states.iter().enumerate() {
                let color = if i == 0 {
                    Color::from(RED)
                } else {
                    Color::from(GREEN)
                };
                match state {
                    FootState::Locked(pos) => {
                        gizmos.sphere(*pos, Quat::IDENTITY, 0.5 * CAPSULE_RADIUS, color);
                    }
                    FootState::Unlocked(info) => {
                        gizmos.sphere(
                            info.pos,
                            Quat::IDENTITY,
                            0.5 * CAPSULE_RADIUS,
                            color.with_luminance(0.2),
                        );
                    }
                }
            }

            if debug_state.get() == &DebugState::Torque {
                gizmos
                    .primitive_3d(
                        &Capsule3d::new(CAPSULE_RADIUS, CAPSULE_HEIGHT - CAPSULE_RADIUS * 2.0),
                        position.clone(),
                        debug.delta_quat,
                        Color::from(GRAY),
                    )
                    .resolution(12);
                gizmos.arrow(
                    *position,
                    *position + debug.delta_quat * Vec3::NEG_Z,
                    Color::WHITE,
                );
                let delta_quat = debug.delta_quat * quat.inverse();
                gizmos.arrow(
                    *position,
                    *position + delta_quat.to_scaled_axis(),
                    Color::BLACK,
                );
            }

            if debug_state.get() == &DebugState::Forces {
                let contact_color = if move_state.slipping {
                    Color::from(RED)
                } else {
                    Color::from(GREEN)
                };
                gizmos.sphere(
                    position.clone() + debug.shape_toi * debug.cast_dir,
                    Quat::IDENTITY,
                    CAST_RADIUS,
                    contact_color,
                );
                gizmos.arrow(
                    position.clone() + debug.contact_point,
                    position.clone() + debug.contact_point + debug.spring_force,
                    Color::from(CYAN_100),
                );
                gizmos.arrow(
                    position.clone() + debug.contact_point,
                    position.clone() + debug.contact_point + debug.normal_force,
                    Color::from(BLUE),
                );

                gizmos.arrow(
                    position.clone() + debug.contact_point,
                    position.clone() + debug.contact_point + debug.tangential_force,
                    Color::from(PINK),
                );

                gizmos.arrow(
                    position.clone() + debug.contact_point,
                    position.clone() + debug.contact_point + debug.tangent_vel,
                    Color::from(ORANGE),
                );
                gizmos.arrow(
                    position.clone() + debug.contact_point,
                    position.clone() + debug.contact_point + debug.target_vel,
                    Color::from(GREEN),
                );
                gizmos.arrow(
                    position.clone() + debug.contact_point,
                    position.clone() + debug.contact_point + debug.target_force,
                    Color::from(RED),
                );
                gizmos.arrow(
                    position.clone() + debug.contact_point + debug.tangential_force,
                    position.clone()
                        + debug.contact_point
                        + debug.tangential_force
                        + debug.angle_force,
                    Color::from(YELLOW),
                );
                gizmos.arrow(
                    position.clone(),
                    position.clone() + debug.spring_torque,
                    Color::from(YELLOW),
                );
            }
        }
        gizmos
            .primitive_3d(
                &Capsule3d::new(CAPSULE_RADIUS, CAPSULE_HEIGHT - CAPSULE_RADIUS * 2.0),
                position.clone(),
                quat.clone(),
                Color::WHITE,
            )
            .resolution(12);
        gizmos.arrow(
            *position,
            *position + 0.2 * move_state.forward_dir,
            Color::from(BLUE),
        );
        gizmos.arrow(
            *position,
            *position - 0.2 * move_state.forward_dir,
            Color::from(RED),
        );
    }
}

pub struct PlayerPlugin;

impl Plugin for PlayerPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(InputManagerPlugin::<PlayerAction>::default());
        app.register_type::<PlayerSpawn>();
        app.register_type::<DistanceJoint>();
        app.register_type::<PlayerGroundSpring>();
        app.register_type::<PlayerAngularSpring>();
        app.insert_state(DebugState::None);
        app.insert_resource(SubstepCount(12));
        app.insert_resource(PlayerGroundSpring {
            rest_length: 0.0,
            stiffness: 0.0,
            min_damping: 0.0,
            max_damping: 0.0,
            max_force: 20.0 * 10.0,
            min_force: 0.0,
        });
        app.insert_resource(PlayerAngularSpring {
            stiffness: 0.,
            damping: 0.0,
            turn_stiffness: 0.0,
        });
        app.add_systems(Startup, (spawn_camera_3rd_person, spawn_camera_1st_person));
        app.add_systems(
            Update,
            (
                spawn_player,
                player_controls,
                respawn_player,
                toggle_active_view,
                draw_debug_gizmos.run_if(not(in_state(DebugState::None))),
                update_procedural_steps.after(PhysicsSet::Sync),
                (track_camera_3rd_person, track_camera_1st_person)
                    .chain()
                    .after(PhysicsSet::Sync)
                    .run_if(not(in_state(DebugState::None))),
            ),
        );
        app.add_systems(
            SubstepSchedule,
            ((
                (track_camera_3rd_person, track_camera_1st_person)
                    .chain()
                    .run_if(in_state(DebugState::None)),
                physics::update_ground_force,
            )
                .chain())
            .before(IntegrationSet::Velocity),
        );
        app.add_systems(OnEnter(DebugState::None), set_visible::<true>);
        app.add_systems(OnExit(DebugState::None), set_visible::<false>);
        // app.add_plugins(PhysicsDebugPlugin::default());
    }
}

// tests for quaternions

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_delta() {
        let from = Quat::from_mat3(&Mat3::from_cols(Vec3::X, Vec3::Y, Vec3::Z));
        let target = Quat::from_mat3(&Mat3::from_cols(Vec3::NEG_X, Vec3::NEG_Y, Vec3::Z));
        let delta = target * from.inverse();
        println!("{:?}", delta.to_scaled_axis());
        assert!(false);
    }
}
