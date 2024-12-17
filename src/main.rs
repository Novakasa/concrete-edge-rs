use std::{env, fmt::Debug};

use avian3d::prelude::*;
use bevy::{
    pbr::{ExtendedMaterial, MaterialExtension},
    prelude::*,
    render::render_resource::{AsBindGroup, ShaderRef},
    window::{CursorGrabMode, PrimaryWindow},
};
use bevy_inspector_egui::quick::WorldInspectorPlugin;
use blenvy::{
    blueprints::spawn_from_blueprints::{
        BlueprintInfo, GameWorldTag, HideUntilReady, SpawnBlueprint,
    },
    BlenvyPlugin,
};
use leafwing_input_manager::prelude::*;
use player::{DebugState, Player};

mod player;
mod util;

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
        input_map.insert(Self::PhysicsSpeedSlower, MouseScrollDirection::DOWN);
        input_map.insert(Self::PhysicsSpeedFaster, MouseScrollDirection::UP);
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

#[derive(Asset, TypePath, AsBindGroup, Debug, Clone)]
struct DebugMaterial {}

impl MaterialExtension for DebugMaterial {
    fn fragment_shader() -> ShaderRef {
        "shaders/debug_material.wgsl".into()
    }

    fn deferred_fragment_shader() -> ShaderRef {
        "shaders/debug_material.wgsl".into()
    }
}

#[derive(Component, Reflect, Debug)]
#[reflect(Component)]
struct DebugMaterialMarker;

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
            DebugState::None => next_debug_state.set(DebugState::Colliders),
            DebugState::Colliders => next_debug_state.set(DebugState::Forces),
            DebugState::Forces => next_debug_state.set(DebugState::Torque),
            DebugState::Torque => next_debug_state.set(DebugState::None),
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

fn load_level(mut commands: Commands) {
    commands.spawn((
        BlueprintInfo::from_path("levels/World.glb"),
        SpawnBlueprint,
        HideUntilReady,
        GameWorldTag,
    ));
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

fn replace_platform_material(
    mut commands: Commands,
    query: Query<
        (Entity, &MeshMaterial3d<StandardMaterial>),
        (Without<Player>, Without<DebugMaterialMarker>),
    >,
    mut materials: ResMut<Assets<ExtendedMaterial<StandardMaterial, DebugMaterial>>>,
    standard_materials: Res<Assets<StandardMaterial>>,
) {
    for (entity, prev_material) in query.iter() {
        let color = standard_materials.get(prev_material).unwrap().base_color;
        println!("Replacing platform material: {:?}", entity);
        commands.entity(entity).insert(DebugMaterialMarker);
        commands
            .entity(entity)
            .insert(MeshMaterial3d(materials.add(ExtendedMaterial {
                base: StandardMaterial::from(color),
                extension: DebugMaterial {},
            })));
        commands
            .entity(entity)
            .remove::<MeshMaterial3d<StandardMaterial>>();
    }
}

fn setup_platforms(
    mut commands: Commands,
    query: Query<(Entity, &Platform, &Children), Without<RigidBody>>,
) {
    for (entity, _platform, children) in query.iter() {
        // println!("Generating collider for platform: {:?}", entity);
        commands
            .entity(entity)
            .insert(Name::new(format!("Platform{}", entity.index())))
            .insert(RigidBody::Static);
        for child in children.iter() {
            commands
                .entity(*child)
                .insert(ColliderConstructor::default());
        }
    }
}

fn lock_cursor(mut q_window: Query<&mut Window, With<PrimaryWindow>>) {
    let mut primary_window = q_window.single_mut();
    primary_window.cursor_options.grab_mode = CursorGrabMode::Locked;
    primary_window.cursor_options.visible = false;
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
        // .add_plugins(LogDiagnosticsPlugin::default())
        // .add_plugins(FrameTimeDiagnosticsPlugin::default())
        .add_plugins(InputManagerPlugin::<GlobalAction>::default())
        .add_plugins(DefaultPlugins)
        .add_plugins(PhysicsPlugins::default())
        .add_plugins(bevy_framepace::FramepacePlugin)
        .add_plugins(BlenvyPlugin::default())
        .add_plugins(player::PlayerPlugin)
        .add_plugins(WorldInspectorPlugin::new())
        .add_plugins(MaterialPlugin::<
            ExtendedMaterial<StandardMaterial, DebugMaterial>,
        >::default())
        .add_systems(Startup, load_level)
        .add_systems(Update, (setup_platforms, setup_player))
        //.add_systems(Update, print_platforms)
        .add_systems(
            Update,
            (
                quit_on_menu,
                physics_speed_control,
                toggle_debug_state,
                replace_platform_material,
            ),
        )
        .add_systems(Startup, lock_cursor)
        .run();
}
