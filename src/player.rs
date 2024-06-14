use bevy::prelude::*;
use bevy_xpbd_3d::prelude::*;

#[derive(Component, Reflect, Debug)]
#[reflect(Component)]
pub struct Player {
    test: i32,
}

#[derive(Component, Debug)]
struct PlayerSpawned;

#[derive(Component, Reflect, Debug)]
struct PlayerFeet;

fn spawn_player(
    mut commands: Commands,
    query: Query<(Entity, &Transform), (With<Player>, Without<PlayerSpawned>)>,
) {
    for (entity, transform) in query.iter() {
        commands.entity(entity).insert(PlayerSpawned);
        println!("Spawning player: {:?}", entity);
        let body = commands
            .spawn((
                Collider::capsule(2.0, 1.0),
                RigidBody::default(),
                Name::new("PlayerBody"),
                Position::from(transform.translation),
            ))
            .id();
        let mut feet_transform = transform.clone();
        feet_transform.translation.y -= 2.5;
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
        let mut spring = DistanceJoint::new(body, feet)
            .with_rest_length(4.5)
            .with_linear_velocity_damping(0.0);
        spring.compliance = 0.001;
        // commands.spawn((spring, Name::new("PlayerSpring")));
        let mut prism = PrismaticJoint::new(body, feet).with_free_axis(Vec3::Y);
        prism.compliance = 0.0005;
        // commands.spawn(prism);
        let mut fixed = FixedJoint::new(body, feet)
            .with_local_anchor_1(-Vec3::Y * 3.5)
            .with_angular_velocity_damping(100.0);
        fixed.compliance = 0.001;
        commands.spawn(fixed);
    }
}

pub struct PlayerPlugin;

impl Plugin for PlayerPlugin {
    fn build(&self, app: &mut App) {
        app.register_type::<Player>();
        app.register_type::<DistanceJoint>();
        app.insert_resource(SubstepCount(12));
        app.add_systems(Update, spawn_player);
    }
}
