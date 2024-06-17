use bevy::prelude::*;
use bevy_xpbd_3d::{prelude::*, SubstepSchedule, SubstepSet};
use leafwing_input_manager::prelude::*;

#[derive(Actionlike, PartialEq, Eq, Hash, Clone, Debug, Reflect)]
pub enum Action {
    Jump,
    Move,
    View,
}

impl Action {
    fn default_input_map() -> InputMap<Self> {
        let mut input_map = InputMap::default();
        input_map.insert(Self::Move, VirtualDPad::wasd());
        input_map.insert(Self::Jump, KeyCode::Space);
        input_map
    }
}

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

#[derive(Component, Reflect, Debug)]
struct PlayerAngularSpring {
    stiffness: f32,
    damping: f32,
}

#[derive(Component, Reflect, Debug, Default)]
struct PlayerMoveState {
    acc_dir: Vec3,
    spring_height: f32,
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
                ExternalTorque::default().with_persistence(false),
                PlayerGroundSpring {
                    rest_length: 2.0,
                    stiffness: 3000.0,
                    damping: 100.0,
                },
                PlayerAngularSpring {
                    stiffness: 1000.0,
                    damping: 100.0,
                },
                InputManagerBundle::<Action>::with_map(Action::default_input_map()),
                PlayerMoveState::default(),
            ))
            .id();
    }
}

fn player_controls(mut query: Query<(&ActionState<Action>, &mut PlayerMoveState), With<Player>>) {
    for (action_state, mut move_state) in query.iter_mut() {
        let dir = action_state
            .clamped_axis_pair(&Action::Move)
            .unwrap()
            .xy()
            .normalize_or_zero();
        println!("Move: {:?}", dir);
        move_state.acc_dir = Vec3::new(dir.x, 0.0, dir.y);
        if action_state.pressed(&Action::Jump) {
            move_state.spring_height = 2.0;
        } else {
            move_state.spring_height = 3.0;
        }
    }
}

fn update_ground_force(
    mut query: Query<
        (
            &RigidBody,
            &mut ExternalForce,
            &mut ExternalTorque,
            &Position,
            &Rotation,
            &LinearVelocity,
            &AngularVelocity,
            &PlayerGroundSpring,
            &PlayerAngularSpring,
            &PlayerMoveState,
        ),
        With<Player>,
    >,
    shape_cast: SpatialQuery,
) {
    for (
        body,
        mut force,
        mut torque,
        Position(position),
        Rotation(quat),
        LinearVelocity(velocity),
        AngularVelocity(angular_vel),
        spring,
        angular_spring,
        move_state,
    ) in query.iter_mut()
    {
        let from_up = *quat * Vec3::Y;
        if let Some(coll) = shape_cast.cast_shape(
            &Collider::sphere(1.0),
            position.clone(),
            Quat::default(),
            Direction3d::new_unchecked(-from_up),
            4.0,
            false,
            SpatialQueryFilter::from_mask(Layer::Platform),
        ) {
            let contact_point = coll.point2 + -from_up * coll.time_of_impact;
            // println!("Time {:?}", coll.time_of_impact);
            let spring_force = (-spring.stiffness
                * (coll.time_of_impact - move_state.spring_height)
                - spring.damping * velocity.y)
                .max(0.0);
            force.clear();
            force.apply_force_at_point(spring_force * Vec3::Y, contact_point, Vec3::ZERO);
            // println!("Force {:?}", force.force());
            let yaw = move_state.acc_dir.z.atan2(move_state.acc_dir.x);
            let pitch = move_state.acc_dir.length();
            let target_quat = Quat::from_euler(EulerRot::YZX, yaw, -0.2 * pitch, 0.0);
            let target_up = target_quat * Vec3::Y;
            let delta_angle = from_up.angle_between(target_up);
            let delta_axis = from_up.cross(target_up).normalize_or_zero();
            //println!("From up dir: {:?}, Target up dir: {:?}", from_up, target_up);
            // println!("Delta angle: {:?}", delta_angle);
            let spring_torque = angular_spring.stiffness * delta_axis * delta_angle
                - (angular_spring.damping * angular_vel.clone());
            torque.set_torque(spring_torque);
            force.apply_force(-spring_torque.cross(contact_point));
        }
    }
}

pub struct PlayerPlugin;

impl Plugin for PlayerPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(InputManagerPlugin::<Action>::default());
        app.register_type::<PlayerSpawn>();
        app.register_type::<DistanceJoint>();
        app.insert_resource(SubstepCount(4));
        app.add_systems(Update, (spawn_player, player_controls));
        app.add_systems(
            SubstepSchedule,
            update_ground_force.before(SubstepSet::Integrate),
        );
    }
}
