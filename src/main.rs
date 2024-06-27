use std::env;

use bevy::{
    core_pipeline::core_2d::graph::input,
    prelude::*,
    window::{CursorGrabMode, PrimaryWindow},
};
use bevy_gltf_components::ComponentsFromGltfPlugin;
use bevy_inspector_egui::quick::WorldInspectorPlugin;
use bevy_registry_export::ExportRegistryPlugin;
use bevy_xpbd_3d::prelude::*;
use leafwing_input_manager::prelude::*;
use player::DebugState;

mod player;

#[derive(Actionlike, PartialEq, Eq, Hash, Clone, Debug, Reflect)]
pub enum GlobalAction {
    Menu,
    PhysicsSpeedSlower,
    PhysicsSpeedFaster,
    PhysicsSpeedReset,
    ToggleDebug,
}

impl GlobalAction {
    fn default_input_map() -> InputMap<Self> {
        let mut input_map = InputMap::default();
        input_map.insert(Self::Menu, KeyCode::Escape);
        input_map.insert(Self::PhysicsSpeedSlower, MouseWheelDirection::Down);
        input_map.insert(Self::PhysicsSpeedFaster, MouseWheelDirection::Up);
        input_map.insert(Self::PhysicsSpeedReset, KeyCode::Digit0);
        input_map.insert(Self::ToggleDebug, KeyCode::F3);
        input_map
    }
}

#[derive(Component, Reflect, Debug)]
#[reflect(Component)]
struct Platform {
    test: i32,
}

#[derive(Component, Reflect, Debug)]
#[reflect(Component)]
struct TestPlayer {
    test: i32,
}

fn quit_on_menu(
    mut commands: Commands,
    input: Res<ActionState<GlobalAction>>,
    q_window: Query<Entity, With<PrimaryWindow>>,
) {
    if input.just_pressed(&GlobalAction::Menu) {
        println!("Quitting on menu");
        for window in q_window.iter() {
            commands.entity(window).despawn();
        }
    }
}

fn toggle_debug_state(
    mut next_debug_state: ResMut<NextState<DebugState>>,
    debug_state: Res<State<DebugState>>,
    input: Res<ActionState<GlobalAction>>,
) {
    if input.just_pressed(&GlobalAction::ToggleDebug) {
        match debug_state.get() {
            DebugState::None => next_debug_state.set(DebugState::On),
            DebugState::On => next_debug_state.set(DebugState::None),
        }
    }
}

fn physics_speed_control(mut time: ResMut<Time<Physics>>, input: Res<ActionState<GlobalAction>>) {
    let relative_speed = time.relative_speed();
    if input.just_pressed(&GlobalAction::PhysicsSpeedSlower) {
        time.set_relative_speed((relative_speed / 2.0).max(0.01));
        println!("Physics speed: {}", time.relative_speed());
    }
    if input.just_pressed(&GlobalAction::PhysicsSpeedFaster) {
        time.set_relative_speed((relative_speed * 2.0).min(1.0));
        println!("Physics speed: {}", time.relative_speed());
    }
    if input.just_pressed(&GlobalAction::PhysicsSpeedReset) {
        time.set_relative_speed(1.0);
        println!("Physics speed: {}", time.relative_speed());
    }
}

fn load_level(mut commands: Commands, asset_server: Res<AssetServer>) {
    commands
        .spawn(SceneBundle {
            scene: asset_server.load("Scene.glb#Scene0"),
            ..Default::default()
        })
        .insert(Name::new("LevelScene"));
}

fn print_platforms(query: Query<(&Platform, &Transform)>) {
    println!("Platforms:");
    for (platform, transform) in query.iter() {
        println!("{:?}", platform);
        println!("{:?}", transform);
    }
}

fn setup_player(
    mut commands: Commands,
    query: Query<Entity, (With<TestPlayer>, Without<ColliderDensity>)>,
) {
    for entity in query.iter() {
        println!("Setting up player: {:?}", entity);
        commands
            .entity(entity)
            .insert((Collider::cuboid(3.0, 1.0, 2.0), RigidBody::default()))
            .insert(Name::new("TestPlayer"));
    }
}

fn setup_platforms(
    mut commands: Commands,
    query: Query<(Entity, &Platform, &Children), Without<RigidBody>>,
) {
    for (entity, platform, children) in query.iter() {
        println!("Generating collider for platform: {:?}", entity);
        commands
            .entity(entity)
            .insert(Name::new(format!("Platform{}", entity.index())))
            .insert(RigidBody::Static);
        for child in children.iter() {
            commands.entity(*child).insert(AsyncCollider::default());
        }
    }
}

fn lock_cursor(mut q_window: Query<&mut Window, With<PrimaryWindow>>) {
    let mut primary_window = q_window.single_mut();
    primary_window.cursor.grab_mode = CursorGrabMode::Locked;
    primary_window.cursor.visible = false;
}

fn main() {
    env::set_var("RUST_BACKTRACE", "1");
    env::set_var("RUST_LOG", "pybricks_ble=info,brickrail=info");
    App::new()
        .register_type::<Platform>()
        .register_type::<TestPlayer>()
        .insert_resource(Gravity::default())
        .init_resource::<ActionState<GlobalAction>>()
        .insert_resource(GlobalAction::default_input_map())
        .add_plugins(InputManagerPlugin::<GlobalAction>::default())
        .add_plugins(DefaultPlugins)
        .add_plugins(PhysicsPlugins::default())
        .add_plugins(bevy_framepace::FramepacePlugin)
        // .add_plugins(PhysicsDebugPlugin::default())
        .add_plugins(ComponentsFromGltfPlugin { legacy_mode: false })
        .add_plugins(player::PlayerPlugin)
        .add_plugins(WorldInspectorPlugin::new())
        .add_systems(Startup, load_level)
        .add_systems(Update, (setup_platforms, setup_player))
        //.add_systems(Update, print_platforms)
        .add_plugins(ExportRegistryPlugin::default())
        .add_systems(
            Update,
            (quit_on_menu, physics_speed_control, toggle_debug_state),
        )
        .add_systems(Startup, lock_cursor)
        .run();
}
