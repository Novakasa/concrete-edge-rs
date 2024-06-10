use bevy::prelude::*;
use bevy_gltf_components::ComponentsFromGltfPlugin;
use bevy_registry_export::ExportRegistryPlugin;

#[derive(Component, Reflect, Debug)]
#[reflect(Component)]
struct Platform {
    test: i32,
}

fn load_level(mut commands: Commands, asset_server: Res<AssetServer>) {
    commands.spawn(SceneBundle {
        scene: asset_server.load("Scene.glb#Scene0"),
        ..Default::default()
    });
}

fn print_platforms(query: Query<(&Platform, &Transform)>) {
    println!("Platforms:");
    for (platform, transform) in query.iter() {
        println!("{:?}", platform);
        println!("{:?}", transform);
    }
}

fn main() {
    App::new()
        .register_type::<Platform>()
        .add_plugins(DefaultPlugins)
        .add_plugins(ExportRegistryPlugin::default())
        .add_plugins(ComponentsFromGltfPlugin::default())
        .add_systems(Startup, load_level)
        //.add_systems(Update, print_platforms)
        .run();
}
