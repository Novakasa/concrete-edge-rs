use std::{env, fmt::Debug};

use avian3d::prelude::*;
use bevy::{
    color::palettes::tailwind::SKY_300,
    ecs::system::SystemState,
    pbr::{ExtendedMaterial, MaterialExtension, NotShadowCaster},
    prelude::*,
    render::render_resource::{AsBindGroup, ShaderRef},
    window::{CursorGrabMode, PrimaryWindow, WindowMode},
};
use bevy_inspector_egui::{
    bevy_egui::{EguiContext, EguiPlugin},
    egui,
    reflect_inspector::ui_for_value,
};
use blenvy::{
    blueprints::spawn_from_blueprints::{
        BlueprintInfo, GameWorldTag, HideUntilReady, SpawnBlueprint,
    },
    BlenvyPlugin,
};
use leafwing_input_manager::prelude::*;
use player::{physics::PhysicsGizmos, PlayerParams, RigGizmos};

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

fn window_settings(// mut window: Single<&mut Window>,
    // mut framepace_settings: ResMut<bevy_framepace::FramepaceSettings>,
) {
    // window.present_mode = PresentMode::AutoNoVsync;
    println!("Window settings");
    // framepace_settings.limiter = Limiter::from_framerate(60.0);
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

fn toggle_gizmos(input: Res<ButtonInput<KeyCode>>, mut config_store: ResMut<GizmoConfigStore>) {
    if input.just_pressed(KeyCode::F3) {
        let (config, _) = config_store.config_mut::<PhysicsGizmos>();
        config.enabled = !config.enabled;
    }
    if input.just_pressed(KeyCode::F4) {
        let (config, _) = config_store.config_mut::<RigGizmos>();
        config.enabled = !config.enabled;
    }
    if input.just_pressed(KeyCode::KeyV) {
        for (_, config, _) in config_store.iter_mut() {
            config.depth_bias = -1.0 - config.depth_bias;
        }
    }
}

fn toggle_mouse_interaction(
    state: Res<State<MouseInteraction>>,
    mut next_state: ResMut<NextState<MouseInteraction>>,
    input: Res<ButtonInput<KeyCode>>,
) {
    if input.just_pressed(KeyCode::Backquote) {
        println!("Toggling mouse interaction");
        next_state.set(if state.get() == &MouseInteraction::Camera {
            MouseInteraction::Inspector
        } else {
            MouseInteraction::Camera
        });
    }
}

fn toggle_fullscreen(
    mut q_window: Query<&mut Window, With<PrimaryWindow>>,
    input: Res<ButtonInput<KeyCode>>,
) {
    if let Ok(mut primary_window) = q_window.get_single_mut() {
        if input.just_pressed(KeyCode::F11) {
            primary_window.mode = match primary_window.mode {
                WindowMode::BorderlessFullscreen(_) => WindowMode::Windowed,
                _ => WindowMode::BorderlessFullscreen(MonitorSelection::Current),
            };
        }
    }
}

fn physics_speed_control(mut time: ResMut<Time<Physics>>, input: Res<ActionState<GlobalAction>>) {
    let relative_speed = time.relative_speed();
    if input.just_pressed(&GlobalAction::PhysicsSpeedSlower) {
        time.set_relative_speed((relative_speed / 2.0).max(0.001));
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
    q_materials: Query<
        (Entity, &Parent, &MeshMaterial3d<StandardMaterial>),
        Without<DebugMaterialMarker>,
    >,
    q_platforms: Query<Entity, With<Platform>>,
    mut materials: ResMut<Assets<ExtendedMaterial<StandardMaterial, DebugMaterial>>>,
    standard_materials: Res<Assets<StandardMaterial>>,
) {
    for (entity, parent, prev_material) in q_materials.iter() {
        if q_platforms.contains(parent.get()) {
            continue;
        }
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
            .insert(RigidBody::Static)
            .insert(Friction::new(0.0));
        for child in children.iter() {
            commands
                .entity(*child)
                .insert(ColliderConstructor::default());
        }
    }
}

fn lock_cursor(mut q_window: Query<&mut Window, With<PrimaryWindow>>) {
    println!("Locking cursor");
    let mut primary_window = q_window.single_mut();
    primary_window.cursor_options.grab_mode = CursorGrabMode::Locked;
    primary_window.cursor_options.visible = false;
}

fn unlock_cursor(mut q_window: Query<&mut Window, With<PrimaryWindow>>) {
    println!("Unlocking cursor");
    let mut primary_window = q_window.single_mut();
    primary_window.cursor_options.grab_mode = CursorGrabMode::None;
    primary_window.cursor_options.visible = true;
}

fn skybox(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(2.0, 1.0, 1.0))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::from(SKY_300),
            unlit: true,
            cull_mode: None,
            ..default()
        })),
        Transform::from_scale(Vec3::splat(2000.0)),
        NotShadowCaster,
    ));
}

#[derive(States, Debug, Clone, PartialEq, Eq, Hash)]
pub enum MouseInteraction {
    Camera,
    Inspector,
}

fn inspector_ui(world: &mut World) {
    let Ok(egui_context) = world
        .query_filtered::<&mut EguiContext, With<PrimaryWindow>>()
        .get_single(world)
    else {
        return;
    };
    let mut egui_context = egui_context.clone();

    egui::Window::new("UI").show(egui_context.get_mut(), |ui| {
        egui::ScrollArea::vertical().show(ui, |ui| {
            let mut state = SystemState::<(ResMut<PlayerParams>, Res<AppTypeRegistry>)>::new(world);
            let (mut params, registry) = state.get_mut(world);

            ui.collapsing("Spring", |ui| {
                ui_for_value(&mut params.springs, ui, &registry.read());
            });
            ui.collapsing("Physics", |ui| {
                ui_for_value(&mut params.physics, ui, &registry.read());
            });
            ui.collapsing("input", |ui| {
                ui_for_value(&mut params.input, ui, &registry.read());
            });
            if ui.button("Save").clicked() {
                params.save();
            }
        });
    });
}

fn main() {
    env::set_var("RUST_BACKTRACE", "1");
    env::set_var("RUST_LOG", "pybricks_ble=info,brickrail=info");
    App::new()
        .register_type::<Platform>()
        .register_type::<TestPlayer>()
        .add_plugins(DefaultPlugins)
        .add_plugins(EguiPlugin)
        .add_plugins(bevy_inspector_egui::DefaultInspectorConfigPlugin)
        .insert_resource(Gravity::default())
        .init_resource::<ActionState<GlobalAction>>()
        .insert_resource(GlobalAction::default_input_map())
        .insert_resource(AmbientLight {
            color: Color::WHITE,
            brightness: 50.0,
        })
        .insert_state(MouseInteraction::Camera)
        // .add_plugins(LogDiagnosticsPlugin::default())
        // .add_plugins(FrameTimeDiagnosticsPlugin::default())
        .add_plugins(InputManagerPlugin::<GlobalAction>::default())
        // .add_plugins(ScreenSpaceAmbientOcclusionPlugin)
        .add_plugins(PhysicsPlugins::default())
        .add_plugins(bevy_framepace::FramepacePlugin)
        .add_plugins(BlenvyPlugin::default())
        .add_plugins(player::PlayerPlugin)
        // .add_plugins(WorldInspectorPlugin::new())
        .add_plugins(MaterialPlugin::<
            ExtendedMaterial<StandardMaterial, DebugMaterial>,
        >::default())
        .add_systems(Startup, (load_level, window_settings, skybox))
        .add_systems(Update, (setup_platforms, setup_player))
        .add_systems(
            Update,
            (
                quit_on_menu,
                physics_speed_control,
                toggle_gizmos,
                toggle_mouse_interaction,
                toggle_fullscreen,
                replace_platform_material,
                inspector_ui,
            ),
        )
        .add_systems(OnEnter(MouseInteraction::Camera), lock_cursor)
        .add_systems(OnExit(MouseInteraction::Camera), unlock_cursor)
        .run();
}
