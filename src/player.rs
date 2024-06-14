use bevy::prelude::*;
use bevy_xpbd_3d::prelude::*;

#[derive(Component, Reflect, Debug)]
#[reflect(Component)]
pub struct Player {
    test: i32,
}

#[derive(Component, Reflect, Debug)]
struct PlayerFeet;

fn spawn_player(
    mut commands: Commands,
    query: Query<(Entity, &Transform), (With<Player>, Without<RigidBody>)>,
) {
    for (entity, transform) in query.iter() {
        println!("Spawning player: {:?}", entity);
        commands.entity(entity).insert((
            Collider::capsule(2.0, 1.0),
            RigidBody::default(),
            Name::new("Player"),
        ));
        let mut feet_transform = transform.clone();
        feet_transform.translation.y -= 1.5;
        println!("transform: {:?}", feet_transform);
        let feet = commands
            .spawn((
                Collider::sphere(1.0),
                RigidBody::default(),
                Name::new("PlayerFeet"),
                PlayerFeet,
                Position::from(feet_transform.translation),
            ))
            .id();
        let mut joint = DistanceJoint::new(entity, feet)
            .with_rest_length(4.5)
            .with_linear_velocity_damping(14.0);
        joint.compliance = 0.001;
        commands.spawn(joint);
    }
}

pub struct PlayerPlugin;

impl Plugin for PlayerPlugin {
    fn build(&self, app: &mut App) {
        app.register_type::<Player>();
        app.register_type::<DistanceJoint>();
        app.add_systems(Update, spawn_player);
    }
}
