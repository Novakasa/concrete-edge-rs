use bevy::prelude::*;
use bevy_xpbd_3d::{prelude::*, SubstepSchedule, SubstepSet};

#[derive(PhysicsLayer)]
pub enum Layer {
    Player,
    Platform,
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
struct Player;

#[derive(Component, Reflect, Debug)]
struct PlayerGroundSpring {
    rest_length: f32,
    stiffness: f32,
    damping: f32,
}

fn spawn_player(
    mut commands: Commands,
    query: Query<(Entity, &Transform), (With<PlayerSpawn>, Without<PlayerSpawned>)>,
) {
    for (entity, transform) in query.iter() {
        commands.entity(entity).insert(PlayerSpawned);
        println!("Spawning player: {:?}", entity);
        let body = commands
            .spawn((
                Collider::capsule(2.0, 1.0),
                CollisionLayers::new(Layer::Player, Layer::Platform),
                RigidBody::default(),
                Name::new("PlayerBody"),
                Position::from(transform.translation),
                Player,
                ExternalForce::default().with_persistence(false),
                PlayerGroundSpring {
                    rest_length: 2.0,
                    stiffness: 1000.0,
                    damping: 100.0,
                },
            ))
            .id();
    }
}

fn update_ground_force(
    mut query: Query<
        (
            &RigidBody,
            &mut ExternalForce,
            &Position,
            &LinearVelocity,
            &PlayerGroundSpring,
        ),
        With<Player>,
    >,
    shape_cast: SpatialQuery,
) {
    for (body, mut force, Position(position), LinearVelocity(velocity), spring) in query.iter_mut()
    {
        if let Some(coll) = shape_cast.cast_shape(
            &Collider::sphere(1.0),
            position.clone(),
            Quat::default(),
            Direction3d::NEG_Y,
            4.0,
            false,
            SpatialQueryFilter::from_mask(Layer::Platform),
        ) {
            println!("Time {:?}", coll.time_of_impact);
            force.set_force(
                (-spring.stiffness * (coll.time_of_impact - spring.rest_length)
                    - spring.damping * velocity.y)
                    * Vec3::Y,
            );
            println!("Force {:?}", force.force());
        }
    }
}

pub struct PlayerPlugin;

impl Plugin for PlayerPlugin {
    fn build(&self, app: &mut App) {
        app.register_type::<PlayerSpawn>();
        app.register_type::<DistanceJoint>();
        app.insert_resource(SubstepCount(12));
        app.add_systems(Update, spawn_player);
        app.add_systems(
            SubstepSchedule,
            update_ground_force.before(SubstepSet::Integrate),
        );
    }
}
