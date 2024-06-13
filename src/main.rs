use bevy::prelude::*;
use bevy_gltf_components::ComponentsFromGltfPlugin;
use bevy_inspector_egui::{quick::WorldInspectorPlugin, DefaultInspectorConfigPlugin};
use bevy_registry_export::ExportRegistryPlugin;
use bevy_xpbd_3d::prelude::*;

#[derive(Component, Reflect, Debug)]
#[reflect(Component)]
struct Platform {
    test: i32,
}

#[derive(Component, Reflect, Debug)]
#[reflect(Component)]
struct Player {
    test: i32,
}

#[derive(Component)]
struct Processed;

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
    query: Query<Entity, (With<Player>, Without<ColliderDensity>)>,
) {
    for entity in query.iter() {
        println!("Setting up player: {:?}", entity);
        commands
            .entity(entity)
            .insert((Collider::cuboid(3.0, 1.0, 2.0), RigidBody::default()))
            .insert(Name::new("Player"));
    }
}

fn setup_platforms(
    mut commands: Commands,
    query: Query<(Entity, &Platform, &Children), Without<Processed>>,
) {
    for (entity, platform, children) in query.iter() {
        println!("Generating collider for platform: {:?}", entity);
        commands
            .entity(entity)
            .insert(Name::new(format!("Platform{}", entity.index())))
            .insert(Processed)
            .insert(RigidBody::Static);
        for child in children.iter() {
            commands.entity(*child).insert(AsyncCollider::default());
        }
    }
}

fn main() {
    App::new()
        .register_type::<Platform>()
        .register_type::<Player>()
        .add_plugins(DefaultPlugins)
        .add_plugins(ExportRegistryPlugin::default())
        .add_plugins(PhysicsPlugins::default())
        .add_plugins(ComponentsFromGltfPlugin { legacy_mode: false })
        .add_plugins(WorldInspectorPlugin::new())
        .add_systems(Startup, load_level)
        .add_systems(Update, (setup_platforms, setup_player))
        //.add_systems(Update, print_platforms)
        .run();
}
