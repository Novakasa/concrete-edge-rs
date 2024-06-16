use bevy::prelude::*;
use bevy_gltf_components::ComponentsFromGltfPlugin;
use bevy_inspector_egui::{quick::WorldInspectorPlugin, DefaultInspectorConfigPlugin};
use bevy_registry_export::ExportRegistryPlugin;
use bevy_xpbd_3d::prelude::*;
use leafwing_input_manager::InputManagerBundle;

mod player;

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

fn main() {
    App::new()
        .register_type::<Platform>()
        .register_type::<TestPlayer>()
        .insert_resource(Gravity::default())
        .add_plugins(DefaultPlugins)
        .add_plugins(PhysicsPlugins::default())
        .add_plugins(PhysicsDebugPlugin::default())
        .add_plugins(ComponentsFromGltfPlugin { legacy_mode: false })
        .add_plugins(player::PlayerPlugin)
        .add_plugins(WorldInspectorPlugin::new())
        .add_systems(Startup, load_level)
        .add_systems(Update, (setup_platforms, setup_player))
        //.add_systems(Update, print_platforms)
        .add_plugins(ExportRegistryPlugin::default())
        .run();
}
