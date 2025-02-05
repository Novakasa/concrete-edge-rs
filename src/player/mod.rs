use std::f32::consts::PI;

use animation::ProceduralRigState;
use avian3d::prelude::*;
use bevy::{
    color::palettes::{
        css::{BLUE, GREEN, ORANGE, PINK, RED, YELLOW},
        tailwind::CYAN_100,
    },
    prelude::*,
};
use camera::{CameraAnchor1stPerson, CameraAnchor3rdPerson};
use leafwing_input_manager::prelude::*;
use physics::{
    PhysicsDebugInfo, PhysicsGizmos, PhysicsParams, PhysicsState, PlayerAngularSpring,
    PlayerGroundSpring, SpringParams, CAPSULE_HEIGHT, CAPSULE_RADIUS, CAST_RADIUS,
};
use rig::RigBone;
use serde::{Deserialize, Serialize};

use crate::MouseInteraction;

pub mod animation;
pub mod camera;
pub mod physics;
pub mod rewind;
pub mod rig;

#[derive(States, Debug, Clone, PartialEq, Eq, Hash)]
pub enum DebugState {
    None,
    Animation,
    Physics,
}

#[derive(Resource, Reflect, Debug, Serialize, Deserialize, Default)]
pub struct PlayerParams {
    #[serde(alias = "physics_params")]
    pub physics: PhysicsParams,
    #[serde(alias = "spring_params")]
    pub springs: SpringParams,
}

impl PlayerParams {
    pub fn new() -> Self {
        let load_path = "player_params.toml";
        if let Ok(toml) = std::fs::read_to_string(load_path) {
            toml::from_str(&toml).unwrap()
        } else {
            warn!("No player params found, using default");
            Self::default()
        }
    }

    pub fn save(&self) {
        let save_path = "player_params.toml";
        let toml = toml::to_string_pretty(self).unwrap();
        std::fs::write(save_path, toml).unwrap();
    }
}

#[derive(PartialEq, Eq, Hash, Clone, Debug, Reflect)]
pub enum PlayerAction {
    Jump,
    Crouch,
    Move,
    Grab,
    ViewMouse,
    Respawn,
    Rewind,
    Menu,
    ViewMode,
}

impl Actionlike for PlayerAction {
    fn input_control_kind(&self) -> InputControlKind {
        match self {
            Self::Move | Self::ViewMouse => InputControlKind::DualAxis,
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
        input_map.insert_dual_axis(Self::ViewMouse, MouseMove::default());
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
        meshes.add(Mesh::from(Capsule3d::new(
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
                CollisionLayers::new(physics::Layer::Player, physics::Layer::Platform),
                RigidBody::default(),
                Position::from(transform.translation()),
                Transform::from_translation(transform.translation()),
                GlobalTransform::default(),
                Player,
                visibility,
                ExternalForce::default().with_persistence(false),
                ExternalTorque::default().with_persistence(false),
                InputManagerBundle::<PlayerAction>::with_map(PlayerAction::default_input_map()),
                physics::PhysicsState::new(),
                physics::PhysicsDebugInfo::default(),
            ))
            .insert((Restitution::new(0.0), Friction::new(0.0)))
            .insert((
                animation::ProceduralRigState::default(),
                rewind::RewindHistory::default(),
                Name::new("PlayerBody"),
                Mass(0.125),
                AngularInertia::new(0.006 * Vec3::new(1.0, 1.0, 1.0)),
            ))
            .id();
    }
}

fn respawn_player(
    query: Query<&Transform, (With<PlayerSpawn>, Without<Player>)>,
    mut player: Query<
        (
            &mut PhysicsState,
            &mut ProceduralRigState,
            &mut Transform,
            &mut LinearVelocity,
            &mut AngularVelocity,
        ),
        With<Player>,
    >,
    input_query: Query<&ActionState<PlayerAction>>,
) {
    for spawn_transform in query.iter() {
        for action_state in input_query.iter() {
            if action_state.just_pressed(&PlayerAction::Respawn) {
                if let Ok((
                    mut physics_state,
                    mut rig_state,
                    mut transform,
                    mut velocity,
                    mut angular_velocity,
                )) = player.get_single_mut()
                {
                    *physics_state = PhysicsState::new();
                    *rig_state = ProceduralRigState::default();
                    *transform = *spawn_transform;
                    *velocity = LinearVelocity::default();
                    *angular_velocity = AngularVelocity::default();
                }
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
    mouse_state: Res<State<MouseInteraction>>,
) {
    for (action_state, mut move_state) in query.iter_mut() {
        let move_input = action_state
            .clamped_axis_pair(&PlayerAction::Move)
            .normalize_or_zero();
        // println!("{:?}", move_input);
        let cam_input = action_state.axis_pair(&PlayerAction::ViewMouse);
        if let Ok(mut cam3) = q_cam3.get_single_mut() {
            if mouse_state.get() == &MouseInteraction::Camera {
                cam3.yaw += cam_input.x * -0.005;
                cam3.pitch = (cam3.pitch + cam_input.y * -0.005).clamp(-0.5 * PI, 0.5 * PI);
            }

            if let Ok(mut cam1) = q_cam1.get_single_mut() {
                if mouse_state.get() == &MouseInteraction::Camera {
                    cam1.yaw = cam3.yaw;
                    cam1.pitch = cam3.pitch;
                }
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
        ),
        With<Player>,
    >,
    mut physics_gizmos: Gizmos<PhysicsGizmos>,
) {
    for (debug, Position(position), Rotation(quat), LinearVelocity(_vel), physics_state) in
        query.iter_mut()
    {
        if let Some(contact) = &physics_state.air_state.predicted_contact {
            physics_gizmos.sphere(
                Isometry3d::from_translation(contact.contact_point),
                0.1,
                Color::from(RED),
            );
        }
        if physics_state.ground_state.contact_point.is_some() {
            let contact_color = if physics_state.ground_state.slipping {
                Color::from(RED)
            } else {
                Color::from(GREEN)
            };
            physics_gizmos.sphere(
                Isometry3d::from_translation(position.clone() + debug.shape_toi * debug.cast_dir),
                CAST_RADIUS,
                contact_color,
            );
            physics_gizmos.arrow(
                position.clone() + debug.contact_point,
                position.clone() + debug.contact_point + debug.spring_force,
                Color::from(CYAN_100),
            );
            physics_gizmos.arrow(
                position.clone() + debug.contact_point,
                position.clone() + debug.contact_point + debug.normal_force,
                Color::from(BLUE),
            );

            physics_gizmos.arrow(
                position.clone() + debug.contact_point,
                position.clone() + debug.contact_point + debug.tangential_force,
                Color::from(PINK),
            );

            physics_gizmos.arrow(
                position.clone() + debug.contact_point,
                position.clone() + debug.contact_point + debug.tangent_vel,
                Color::from(ORANGE),
            );
            physics_gizmos.arrow(
                position.clone() + debug.contact_point,
                position.clone() + debug.contact_point + debug.target_vel,
                Color::from(GREEN),
            );
            physics_gizmos.arrow(
                position.clone() + debug.contact_point,
                position.clone() + debug.contact_point + debug.target_force,
                Color::from(RED),
            );
            physics_gizmos.arrow(
                position.clone() + debug.contact_point + debug.tangential_force,
                position.clone() + debug.contact_point + debug.tangential_force + debug.angle_force,
                Color::from(YELLOW),
            );
            physics_gizmos.arrow(
                position.clone(),
                position.clone() + debug.spring_torque,
                Color::from(YELLOW),
            );
        } else {
            // draw circle at end of cast
            physics_gizmos.sphere(
                Isometry3d::from_translation(
                    position.clone()
                        - (RigBone::max_contact_dist() - CAST_RADIUS)
                            * physics_state.ground_state.neg_cast_vec,
                ),
                CAST_RADIUS,
                Color::BLACK,
            );
        }
        physics_gizmos
            .primitive_3d(
                &Capsule3d::new(CAPSULE_RADIUS, CAPSULE_HEIGHT - CAPSULE_RADIUS * 2.0),
                Isometry3d::new(position.clone(), quat.clone()),
                Color::WHITE,
            )
            .resolution(6);
        physics_gizmos.arrow(
            *position,
            *position + 0.2 * physics_state.forward_dir,
            Color::from(BLUE),
        );
        physics_gizmos.arrow(
            *position,
            *position - 0.2 * physics_state.forward_dir,
            Color::from(RED),
        );
    }
}

#[derive(Debug, Reflect, Default, GizmoConfigGroup)]
pub struct RigGizmos;

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
        app.insert_resource(PlayerParams::new());
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
                camera::toggle_active_view,
                draw_debug_gizmos.after(animation::update_procedural_state),
                (
                    camera::track_camera_3rd_person,
                    camera::track_camera_1st_person,
                )
                    .chain()
                    .after(animation::update_procedural_state)
                    .after(PhysicsSet::Sync),
            ),
        );
        app.add_systems(Update, respawn_player);
        app.add_systems(OnEnter(DebugState::None), set_visible::<true>);
        app.add_systems(OnExit(DebugState::None), set_visible::<false>);
        app.add_plugins(physics::PlayerPhysicsPlugin);
        app.add_plugins(animation::PlayerAnimationPlugin);
        app.add_plugins(rewind::RewindPlugin);
        app.add_plugins(rig::RigPlugin);
        app.insert_gizmo_config(
            RigGizmos::default(),
            GizmoConfig {
                enabled: false,
                ..Default::default()
            },
        );
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
