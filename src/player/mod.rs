use std::f32::consts::PI;

use animation::{FootState, ProceduralRigState};
use avian3d::prelude::*;
use bevy::{
    color::palettes::{
        css::{BLUE, GRAY, GREEN, ORANGE, PINK, RED, YELLOW},
        tailwind::CYAN_100,
    },
    prelude::*,
};
use camera::{CameraAnchor1stPerson, CameraAnchor3rdPerson};
use dynamics::integrator::IntegrationSet;
use leafwing_input_manager::prelude::*;
use physics::{
    PhysicsDebugInfo, PhysicsState, PlayerAngularSpring, PlayerGroundSpring, PlayerSpringParams,
    CAPSULE_HEIGHT, CAPSULE_RADIUS, CAST_RADIUS, MAX_TOI,
};

use crate::util::ik2_positions;

mod animation;
mod camera;
mod physics;
mod rewind;
mod rig;

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
    Crouch,
    Move,
    Grab,
    View,
    Respawn,
    Rewind,
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
        input_map.insert_dual_axis(Self::Move, VirtualDPad::wasd());
        input_map.insert(Self::Jump, KeyCode::Space);
        input_map.insert(Self::Crouch, MouseButton::Right);
        input_map.insert(Self::Respawn, KeyCode::F2);
        input_map.insert(Self::Rewind, MouseButton::Middle);
        input_map.insert_dual_axis(Self::View, MouseMove::default());
        input_map.insert(Self::Menu, KeyCode::Escape);
        input_map.insert(Self::ViewMode, KeyCode::Tab);
        input_map.insert(Self::Grab, MouseButton::Left);
        input_map
    }
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

fn spawn_player(
    mut commands: Commands,
    query: Query<(Entity, &GlobalTransform), (With<PlayerSpawn>, Without<PlayerSpawned>)>,
    mut meshes: ResMut<Assets<Mesh>>,
    debug_state: Res<State<DebugState>>,
) {
    for (entity, transform) in query.iter() {
        commands.entity(entity).insert(PlayerSpawned);
        println!("Spawning player: {:?}", entity);
        let capsule = meshes.add(Mesh::from(Capsule3d::new(
            physics::CAPSULE_RADIUS,
            physics::CAPSULE_HEIGHT - 2.0 * physics::CAPSULE_RADIUS,
        )));
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
                CollisionLayers::new(physics::Layer::Player, physics::Layer::Platform),
                RigidBody::default(),
                Position::from(transform.translation()),
                Transform::from_translation(transform.translation()),
                GlobalTransform::default(),
                Player,
                visibility,
                ExternalForce::default().with_persistence(false),
                ExternalTorque::default().with_persistence(false),
                PlayerSpringParams::new(),
                InputManagerBundle::<PlayerAction>::with_map(PlayerAction::default_input_map()),
                physics::PhysicsState::new(),
                physics::PhysicsDebugInfo::default(),
            ))
            .insert((Restitution::new(0.0), Friction::new(0.0)))
            .insert((
                animation::ProceduralRigState::default(),
                rewind::RewindHistory::default(),
                Name::new("PlayerBody"),
            ))
            .id();
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
    mut query: Query<(&ActionState<PlayerAction>, &mut physics::PhysicsState), With<Player>>,
    mut q_cam3: Query<&mut CameraAnchor3rdPerson, Without<CameraAnchor1stPerson>>,
    mut q_cam1: Query<&mut CameraAnchor1stPerson, Without<CameraAnchor3rdPerson>>,
    mut next_rewind_state: ResMut<NextState<rewind::RewindState>>,
    rewind_state: Res<State<rewind::RewindState>>,
    mut rewind_info: ResMut<rewind::RewindInfo>,
    time: Res<Time>,
) {
    for (action_state, mut move_state) in query.iter_mut() {
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

            move_state.input_dir = (Quat::from_euler(EulerRot::YXZ, cam3.yaw, 0.0, 0.0)
                * Vec3::new(move_input.x, 0.0, -move_input.y))
            .xz();
            // println!("{:?}", move_state.acc_dir);
            if action_state.just_pressed(&PlayerAction::Jump)
                && move_state.ground_state.contact_point.is_some()
            {
                move_state.ground_state.jumping = true;
            }
            if !action_state.pressed(&PlayerAction::Jump) {
                move_state.ground_state.jumping = false;
            }
            if action_state.just_pressed(&PlayerAction::Rewind) {
                next_rewind_state.set(rewind::RewindState::Rewinding);
            }
            if action_state.just_released(&PlayerAction::Rewind) {
                next_rewind_state.set(rewind::RewindState::Playing);
            }

            if rewind_state.get() == &rewind::RewindState::Rewinding {
                rewind_info.rewind_time += move_input.x * time.delta_secs();
            }

            move_state.grabbing = action_state.pressed(&PlayerAction::Grab);
            move_state.ground_state.crouching = action_state.pressed(&PlayerAction::Crouch);
        }
    }
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
            &LinearVelocity,
            &PhysicsState,
            &ProceduralRigState,
        ),
        With<Player>,
    >,
    mut gizmos: Gizmos,
    debug_state: Res<State<DebugState>>,
) {
    for (debug, Position(position), Rotation(quat), LinearVelocity(_vel), physics_state, steps) in
        query.iter_mut()
    {
        if let Some(contact) = &physics_state.air_state.predicted_contact {
            gizmos.sphere(
                Isometry3d::from_translation(contact.contact_point),
                0.1,
                Color::from(RED),
            );
        }
        if physics_state.ground_state.contact_point.is_some() {
            for (i, state) in steps.ground_state.foot_states.iter().enumerate() {
                let color = if i == 0 {
                    Color::from(RED)
                } else {
                    Color::from(GREEN)
                };
                let pos = match state {
                    FootState::Locked(info) => {
                        gizmos.sphere(
                            Isometry3d::from_translation(info.pos),
                            0.5 * CAPSULE_RADIUS,
                            color,
                        );

                        info.pos
                    }
                    FootState::Unlocked(info) => {
                        gizmos.sphere(
                            Isometry3d::from_translation(info.pos),
                            0.5 * CAPSULE_RADIUS,
                            color.with_luminance(0.2),
                        );
                        info.pos
                    }
                };

                let hip_pos = steps.hip_pos;
                let (pos1, pos2) = ik2_positions(
                    CAPSULE_HEIGHT * 0.4,
                    CAPSULE_HEIGHT * 0.4,
                    pos - hip_pos,
                    physics_state.forward_dir,
                );
                gizmos.arrow(hip_pos, hip_pos + pos1, Color::WHITE);
                gizmos.arrow(hip_pos + pos1, hip_pos + pos2, Color::WHITE);
            }

            if debug_state.get() == &DebugState::Torque {
                gizmos
                    .primitive_3d(
                        &Capsule3d::new(CAPSULE_RADIUS, CAPSULE_HEIGHT - CAPSULE_RADIUS * 2.0),
                        Isometry3d::new(position.clone(), debug.delta_quat),
                        Color::from(GRAY),
                    )
                    .resolution(6);
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
                let contact_color = if physics_state.ground_state.slipping {
                    Color::from(RED)
                } else {
                    Color::from(GREEN)
                };
                gizmos.sphere(
                    Isometry3d::from_translation(
                        position.clone() + debug.shape_toi * debug.cast_dir,
                    ),
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
        } else {
            // draw circle at end of cast
            gizmos.sphere(
                Isometry3d::from_translation(
                    position.clone() - MAX_TOI * physics_state.ground_state.neg_cast_vec,
                ),
                CAST_RADIUS,
                Color::BLACK,
            );
        }
        gizmos
            .primitive_3d(
                &Capsule3d::new(CAPSULE_RADIUS, CAPSULE_HEIGHT - CAPSULE_RADIUS * 2.0),
                Isometry3d::new(position.clone(), quat.clone()),
                Color::WHITE,
            )
            .resolution(6);
        gizmos.arrow(
            *position,
            *position + 0.2 * physics_state.forward_dir,
            Color::from(BLUE),
        );
        gizmos.arrow(
            *position,
            *position - 0.2 * physics_state.forward_dir,
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
        app.insert_resource(SubstepCount(4));
        app.add_systems(
            Startup,
            (
                camera::spawn_camera_3rd_person,
                camera::spawn_camera_1st_person,
            ),
        );
        app.add_systems(
            Update,
            (
                spawn_player,
                player_controls,
                respawn_player,
                camera::toggle_active_view,
                draw_debug_gizmos
                    .run_if(not(in_state(DebugState::None)))
                    .after(animation::update_procedural_state),
                (
                    camera::track_camera_3rd_person,
                    camera::track_camera_1st_person,
                )
                    .chain()
                    .after(PhysicsSet::Sync),
            ),
        );
        app.add_systems(OnEnter(DebugState::None), set_visible::<true>);
        app.add_systems(OnExit(DebugState::None), set_visible::<false>);
        app.add_plugins(physics::PlayerPhysicsPlugin);
        app.add_plugins(animation::PlayerAnimationPlugin);
        app.add_plugins(rewind::RewindPlugin);
        app.add_plugins(rig::RigPlugin);
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
