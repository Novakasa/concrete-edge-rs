use std::f32::consts::PI;

use avian3d::prelude::*;
use bevy::{
    color::palettes::{
        css::{BLUE, GRAY, GREEN, ORANGE, PINK, RED, YELLOW},
        tailwind::CYAN_100,
    },
    ecs::system::SystemParam,
    prelude::*,
};
use dynamics::integrator::IntegrationSet;
use leafwing_input_manager::prelude::*;

const CAPSULE_RADIUS: f32 = 0.2;
const CAPSULE_HEIGHT: f32 = 4.0 * CAPSULE_RADIUS;
const CAST_RADIUS: f32 = 1.0 * CAPSULE_RADIUS;
const MAX_TOI: f32 = CAPSULE_HEIGHT * 1.0;
const FRICTION_MARGIN: f32 = 0.9;
const GLOBAL_FRICTION: f32 = 1.0;

#[derive(Clone)]
enum SpringParams {
    Physical { stiffness: f32, damping: f32 },
    Effective { f: f32, zeta: f32, m: f32 },
}

impl SpringParams {
    fn get_physical(&self) -> Self {
        match self {
            Self::Physical {
                stiffness: _,
                damping: _,
            } => self.clone(),
            Self::Effective { f, zeta, m } => Self::Physical {
                stiffness: (2.0 * PI * f).powi(2) * m,
                damping: zeta * f * m / PI,
            },
        }
    }
}

#[derive(Component, Reflect, Debug, Clone, Default)]
struct SpringValue {
    f: f32,
    zeta: f32,
    m: f32,
    velocity: f32,
    value: f32,
}

impl SpringValue {
    fn update(&mut self, target: f32, dt: f32) -> f32 {
        let k = (2.0 * PI * self.f).powi(2) * self.m;
        let c = self.zeta * self.f * self.m / PI;
        self.velocity += k * (target - self.value) * dt - c * self.velocity * dt;
        self.value += self.velocity * dt;
        self.value
    }
}

#[derive(States, Debug, Clone, PartialEq, Eq, Hash)]
pub enum DebugState {
    None,
    Colliders,
    Torque,
    Forces,
}

#[derive(PartialEq, Eq, Hash, Clone, Debug, Reflect)]
pub enum PlayerAction {
    Jump,
    Move,
    View,
    Respawn,
    Menu,
    ViewMode,
}

impl Actionlike for PlayerAction {
    fn input_control_kind(&self) -> InputControlKind {
        match self {
            Self::Move | Self::View => InputControlKind::DualAxis,
            _ => InputControlKind::Button,
        }
    }
}

impl PlayerAction {
    fn default_input_map() -> InputMap<Self> {
        let mut input_map = InputMap::default();
        input_map.insert_dual_axis(Self::Move, KeyboardVirtualDPad::WASD);
        input_map.insert(Self::Jump, KeyCode::Space);
        input_map.insert(Self::Respawn, KeyCode::KeyR);
        input_map.insert_dual_axis(Self::View, MouseMove::default());
        input_map.insert(Self::Menu, KeyCode::Escape);
        input_map.insert(Self::ViewMode, KeyCode::Tab);
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
pub struct Player;

#[derive(Component, Reflect, Debug, Resource, Clone, Default)]
#[reflect(Component, Resource)]
struct PlayerGroundSpring {
    rest_length: f32,
    stiffness: f32,
    min_damping: f32,
    max_damping: f32,
    max_force: f32,
    min_force: f32,
    smoothing_spring: SpringValue,
}

impl PlayerGroundSpring {
    fn force(&mut self, length: f32, vel: f32, normal: Vec3, dt: f32) -> f32 {
        let damping = self
            .max_damping
            .lerp(self.min_damping, normal.dot(Vec3::Y).abs());
        let target = (-self.stiffness * (length - self.rest_length).min(0.0) - damping * vel)
            .min(self.max_force)
            .max(self.min_force);
        self.smoothing_spring.update(target, dt);
        target
    }
}

#[derive(Component, Reflect, Debug, Resource, Clone, Default)]
#[reflect(Component, Resource)]
struct PlayerAngularSpring {
    stiffness: f32,
    damping: f32,
    turn_stiffness: f32,
}

#[derive(Component, Reflect, Debug, Default)]
struct PlayerMoveState {
    acc_dir: Vec3,
    spring_height: f32,
    prev_vel: Vec3,
    prev_angular_force: Vec3,
    prev_target_force: Vec3,
    neg_cast_vec: Vec3,
    slipping: bool,
    contact_point: Option<Vec3>,
    contact_normal: Option<Vec3>,
    current_force: Vec3,
    ext_force: Vec3,
    forward_dir: Vec3,
}

#[derive(Debug, Default)]
struct FootTravelInfo {
    time: f32,
    pos: Vec3,
    pos0: Vec3,
    duration: f32,
    target: Option<Vec3>,
}

impl FootTravelInfo {
    fn unlocked(pos: Vec3, duration: f32) -> Self {
        Self {
            time: 0.0,
            pos,
            pos0: pos,
            duration,
            target: None,
        }
    }
}

#[derive(Debug)]
enum FootState {
    Locked(Vec3),
    Unlocked(FootTravelInfo),
}

impl FootState {
    fn is_unlocked(&self) -> bool {
        matches!(self, Self::Unlocked(_))
    }
}

impl Default for FootState {
    fn default() -> Self {
        Self::Unlocked(FootTravelInfo::default())
    }
}

#[derive(Component, Debug, Default)]
struct ProceduralRigState {
    hip_pos: Vec3,
    foot_states: [FootState; 2],
}

#[derive(Component, Reflect, Debug, Default)]
struct PhysicsDebugInfo {
    grounded: bool,
    spring_force: Vec3,
    spring_torque: Vec3,
    normal_force: Vec3,
    tangential_force: Vec3,
    shape_toi: f32,
    cast_dir: Vec3,
    spring_dir: Vec3,
    contact_point: Vec3,
    position: Vec3,
    angle_force: Vec3,
    ground_normal: Vec3,
    tangent_vel: Vec3,
    target_vel: Vec3,
    target_force: Vec3,
    delta_quat: Quat,
}

fn spawn_player(
    mut commands: Commands,
    query: Query<(Entity, &GlobalTransform), (With<PlayerSpawn>, Without<PlayerSpawned>)>,
    ground_spring: Res<PlayerGroundSpring>,
    angular_spring: Res<PlayerAngularSpring>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    debug_state: Res<State<DebugState>>,
) {
    for (entity, transform) in query.iter() {
        commands.entity(entity).insert(PlayerSpawned);
        println!("Spawning player: {:?}", entity);
        let capsule = meshes.add(Mesh::from(Capsule3d::new(
            CAPSULE_RADIUS,
            CAPSULE_HEIGHT - 2.0 * CAPSULE_RADIUS,
        )));
        let material = materials.add(Color::WHITE);
        let visibility = if debug_state.get() == &DebugState::None {
            Visibility::Visible
        } else {
            Visibility::Hidden
        };
        let _body = commands
            .spawn((
                Collider::capsule(CAPSULE_RADIUS, CAPSULE_HEIGHT - 2.0 * CAPSULE_RADIUS),
                ColliderDensity(1.5),
                CollisionLayers::new(Layer::Player, Layer::Platform),
                RigidBody::default(),
                Position::from(transform.translation()),
                Transform::from_translation(transform.translation()),
                GlobalTransform::default(),
                Player,
                ExternalForce::default().with_persistence(false),
                ExternalTorque::default().with_persistence(false),
                ground_spring.clone(),
                angular_spring.clone(),
                InputManagerBundle::<PlayerAction>::with_map(PlayerAction::default_input_map()),
                PlayerMoveState {
                    neg_cast_vec: Vec3::Y,
                    ..Default::default()
                },
                PhysicsDebugInfo::default(),
            ))
            .insert(MaterialMeshBundle {
                mesh: capsule,
                material,
                visibility,
                ..Default::default()
            })
            .insert((Restitution::new(0.0), Friction::new(0.0)))
            .insert((ProceduralRigState::default(), Name::new("PlayerBody")))
            .id();
    }
}

#[derive(Component, Reflect, Debug, Default)]
struct CameraAnchor3rdPerson {
    yaw: f32,
    pitch: f32,
}

#[derive(Component, Reflect, Debug, Default)]
struct CameraAnchor1stPerson {
    yaw: f32,
    pitch: f32,
}

fn spawn_camera_3rd_person(mut commands: Commands) {
    let camera_arm = 0.15 * Vec3::new(0.0, 8.0, 25.0);
    let transform =
        Transform::from_translation(camera_arm).looking_to(-camera_arm.normalize(), Vec3::Y);
    commands
        .spawn((
            TransformBundle::default(),
            Name::new("CameraAnchor"),
            CameraAnchor3rdPerson::default(),
        ))
        .with_children(|builder| {
            builder
                .spawn(Camera3dBundle {
                    projection: Projection::Perspective(PerspectiveProjection {
                        fov: PI / 3.0,
                        ..Default::default()
                    }),
                    camera: Camera {
                        is_active: true,
                        ..Default::default()
                    },
                    transform: transform,
                    ..Default::default()
                })
                .insert(Name::new("PlayerCamera"));
        });
}

fn spawn_camera_1st_person(mut commands: Commands) {
    let camera_arm = Vec3::new(0.0, 0.2 * CAPSULE_HEIGHT, 0.0);
    let transform = Transform::from_translation(camera_arm);
    commands
        .spawn((
            TransformBundle::default(),
            Name::new("CameraAnchor"),
            CameraAnchor1stPerson::default(),
        ))
        .with_children(|builder| {
            builder
                .spawn(Camera3dBundle {
                    projection: Projection::Perspective(PerspectiveProjection {
                        fov: PI / 3.0,
                        ..Default::default()
                    }),
                    camera: Camera {
                        is_active: false,
                        ..Default::default()
                    },
                    transform: transform,
                    ..Default::default()
                })
                .insert(Name::new("PlayerCamera"));
        });
}

fn toggle_active_view(
    anchor1: Query<(&CameraAnchor1stPerson, &Children)>,
    anchor3: Query<(&CameraAnchor3rdPerson, &Children)>,
    mut cams: Query<&mut Camera>,
    input_query: Query<&ActionState<PlayerAction>>,
) {
    for input in input_query.iter() {
        if input.just_pressed(&PlayerAction::ViewMode) {
            let mut cam1 = cams
                .get_mut(*anchor1.get_single().unwrap().1.iter().next().unwrap())
                .unwrap();
            cam1.is_active = !cam1.is_active;
            let mut cam3 = cams
                .get_mut(*anchor3.get_single().unwrap().1.iter().next().unwrap())
                .unwrap();
            cam3.is_active = !cam3.is_active;
        }
    }
}

fn track_camera_3rd_person(
    query: Query<&Position, With<Player>>,
    mut camera_query: Query<(&mut Transform, &CameraAnchor3rdPerson)>,
) {
    for Position(pos) in query.iter() {
        for (mut transform, anchor3) in camera_query.iter_mut() {
            transform.translation = pos.clone();
            transform.rotation = Quat::from_euler(EulerRot::YXZ, anchor3.yaw, anchor3.pitch, 0.0);
        }
    }
}

fn track_camera_1st_person(
    query: Query<(&Position, &Rotation), With<Player>>,
    mut camera_query: Query<(&mut Transform, &CameraAnchor1stPerson)>,
) {
    for (Position(pos), Rotation(quat)) in query.iter() {
        let up_dir = *quat * Vec3::Y;
        let pos = pos.clone() + up_dir * CAPSULE_HEIGHT * 0.3;
        for (mut transform, cam1) in camera_query.iter_mut() {
            let view_unrolled = Quat::from_euler(EulerRot::YXZ, cam1.yaw, cam1.pitch, 0.0);
            let forward = view_unrolled * Vec3::NEG_Z;
            *transform = transform.with_translation(pos).looking_to(forward, up_dir);
        }
    }
}

fn respawn_player(
    mut commands: Commands,
    query: Query<Entity, (With<PlayerSpawn>, With<PlayerSpawned>)>,
    player: Query<Entity, With<Player>>,
    input_query: Query<&ActionState<PlayerAction>>,
) {
    for entity in query.iter() {
        for action_state in input_query.iter() {
            if action_state.just_pressed(&PlayerAction::Respawn) {
                if let Ok(player) = player.get_single() {
                    commands.entity(player).despawn_recursive();
                }
                commands.entity(entity).remove::<PlayerSpawned>();
            }
        }
    }
}

fn player_controls(
    mut query: Query<
        (
            &ActionState<PlayerAction>,
            &mut PlayerMoveState,
            &mut PlayerGroundSpring,
            &mut PlayerAngularSpring,
        ),
        With<Player>,
    >,
    mut q_cam3: Query<&mut CameraAnchor3rdPerson, Without<CameraAnchor1stPerson>>,
    mut q_cam1: Query<&mut CameraAnchor1stPerson, Without<CameraAnchor3rdPerson>>,
) {
    for (action_state, mut move_state, mut spring, mut angular_spring) in query.iter_mut() {
        let move_input = action_state
            .clamped_axis_pair(&PlayerAction::Move)
            .normalize_or_zero();
        // println!("{:?}", move_input);
        let cam_input = action_state.axis_pair(&PlayerAction::View);
        if let Ok(mut cam3) = q_cam3.get_single_mut() {
            cam3.yaw += cam_input.x * -0.005;
            cam3.pitch = (cam3.pitch + cam_input.y * -0.005).clamp(-0.5 * PI, 0.5 * PI);

            if let Ok(mut cam1) = q_cam1.get_single_mut() {
                cam1.yaw = cam3.yaw;
                cam1.pitch = cam3.pitch;
            }

            move_state.acc_dir = Quat::from_euler(EulerRot::YXZ, cam3.yaw, 0.0, 0.0)
                * Vec3::new(move_input.x, 0.0, -move_input.y);
            // println!("{:?}", move_state.acc_dir);
            if action_state.pressed(&PlayerAction::Jump) {
                spring.rest_length = CAPSULE_HEIGHT * 1.4 + CAPSULE_RADIUS;
                spring.min_damping = 1.0;
                spring.stiffness = 15.0;
                angular_spring.stiffness = 0.3;
            } else {
                spring.rest_length = CAPSULE_HEIGHT * 0.7 + CAPSULE_RADIUS;
                spring.min_damping = 1.5;
                spring.stiffness = 15.0;
                angular_spring.stiffness = 0.5;
                angular_spring.damping = 0.2;
                angular_spring.turn_stiffness = 0.4;
            }
        }
    }
}

fn update_ground_force(
    mut query: Query<
        (
            &mut ExternalForce,
            &mut ExternalTorque,
            &Position,
            &Rotation,
            &LinearVelocity,
            &AngularVelocity,
            &mut PlayerGroundSpring,
            &PlayerAngularSpring,
            &mut PlayerMoveState,
            &mut PhysicsDebugInfo,
        ),
        With<Player>,
    >,
    shape_cast: SpatialQuery,
    dt: Res<Time<Substeps>>,
    gravity: Res<Gravity>,
) {
    for (
        mut force,
        mut torque,
        Position(position),
        Rotation(quat),
        LinearVelocity(velocity),
        AngularVelocity(angular_vel),
        mut spring,
        angular_spring,
        mut move_state,
        mut debug,
    ) in query.iter_mut()
    {
        let external_forces = gravity.0;
        let ext_dir = external_forces.normalize_or_zero();
        let filter = SpatialQueryFilter::from_mask(Layer::Platform);
        let cast_dir = -move_state.neg_cast_vec;
        let capsule_up = *quat * Vec3::Y;
        let capsule_right = *quat * Vec3::X;
        move_state.forward_dir = *quat * Vec3::NEG_Z;
        if let Some(coll) = shape_cast.cast_shape(
            &Collider::sphere(CAST_RADIUS),
            position.clone(),
            Quat::IDENTITY,
            Dir3::new_unchecked(cast_dir.normalize_or_zero()),
            MAX_TOI,
            false,
            filter.clone(),
        ) {
            let normal = coll.normal1;
            debug.ground_normal = normal;
            move_state.contact_normal = Some(normal);

            let contact_point = coll.point2 + cast_dir * coll.time_of_impact;
            move_state.contact_point = Some(contact_point);
            let spring_dir = -contact_point.normalize_or_zero();
            // let spring_dir = from_up;
            debug.contact_point = contact_point;
            debug.shape_toi = coll.time_of_impact;
            debug.cast_dir = cast_dir;

            let spring_vel = -velocity.dot(normal) / (-spring_dir.dot(normal));
            // println!("Time {:?}", coll.time_of_impact);
            let spring_force = spring.force(
                contact_point.length(),
                spring_vel,
                normal,
                dt.delta_seconds(),
            ) * spring_dir;
            let grounded = spring_force.length() > 0.0001;
            debug.grounded = grounded;
            if !grounded {
                force.clear();
                torque.clear();
                continue;
            }

            debug.spring_force = spring_force;

            let normal_force = spring_force.dot(normal) * normal;
            debug.normal_force = normal_force;
            let friction_force = normal_force.length() * GLOBAL_FRICTION;
            let tangential_force = spring_force - normal_force;
            debug.tangential_force = tangential_force;

            let tangent_plane = normal.cross(Vec3::Y).normalize_or_zero();
            let tangent_slope = normal.cross(tangent_plane).normalize_or_zero();
            let tangent_z = if normal.dot(Vec3::X).abs() > 0.9999 {
                Vec3::Z
            } else {
                Vec3::X.cross(normal).normalize_or_zero()
            };
            let tangent_x = -tangent_z.cross(normal);
            let acc_tangent = move_state.acc_dir.x * tangent_x + move_state.acc_dir.z * tangent_z;
            let tangent_vel = *velocity - velocity.dot(normal) * normal;
            let prev_tangent_vel = move_state.prev_vel - move_state.prev_vel.dot(normal) * normal;

            debug.tangent_vel = tangent_vel;

            let target_vel = acc_tangent * 7.0;
            debug.target_vel = target_vel;
            let denominator = 1.0 - tangent_slope.dot(ext_dir).powi(2);
            let slope_force = if denominator == 0.0 {
                external_forces
            } else {
                -tangent_slope.dot(ext_dir) * normal_force.dot(ext_dir) * tangent_slope
                    / denominator
            };

            // let slope_force = external_forces - normal.dot(external_forces) * normal;

            let mut target_force = 0.3 * (target_vel - tangent_vel);
            if friction_force > 0.0 {
                target_force = (target_force.length() / friction_force).powi(1)
                    * friction_force
                    * target_force.normalize_or_zero()
            }
            target_force -= 0.0 * (tangent_vel - prev_tangent_vel) / dt.delta_seconds();

            if target_force.length() > friction_force * FRICTION_MARGIN {
                target_force = add_results_in_length(
                    target_force.normalize_or_zero(),
                    -slope_force,
                    friction_force * FRICTION_MARGIN,
                )
                .unwrap_or(target_force)
            }
            target_force -= slope_force;
            move_state.prev_vel = velocity.clone();
            debug.target_force = target_force;
            // println!("{:?}, {:?}", target_force, normal_force);

            let target_spring_dir = (target_force + normal_force).normalize_or_zero();

            let raw_neg_cast_vec = (move_state.neg_cast_vec
                + 10.0 * (target_spring_dir - spring_dir) * dt.delta_seconds())
            .normalize_or_zero();

            let quat = Quat::from_rotation_arc(capsule_up, raw_neg_cast_vec);
            let angle = capsule_up.angle_between(raw_neg_cast_vec);
            move_state.neg_cast_vec = if angle < 0.0001 {
                raw_neg_cast_vec
            } else {
                Quat::IDENTITY.slerp(quat, angle.min(0.3 * PI) / angle) * capsule_up
            };

            let target_up = move_state.neg_cast_vec;
            let target_right = target_vel.cross(target_up).try_normalize().unwrap_or(
                move_state
                    .forward_dir
                    .cross(target_up)
                    .try_normalize()
                    .unwrap(),
            );
            let delta_quat = Quat::from_rotation_arc(capsule_up, target_up);

            debug.delta_quat = delta_quat;
            let angular_spring_torque = angular_spring.stiffness
                * Quat::from_rotation_arc(capsule_up, target_up).to_scaled_axis()
                + (angular_spring.turn_stiffness
                    * Quat::from_rotation_arc(capsule_right, target_right).to_scaled_axis())
                - angular_spring.damping * angular_vel.clone();

            debug.spring_torque = angular_spring_torque;

            let angle_force = -normal.cross(angular_spring_torque) / (normal.dot(contact_point));

            debug.angle_force = angle_force;
            move_state.current_force =
                (tangential_force + angle_force).clamp_length_max(friction_force);
            move_state.ext_force = slope_force;
            move_state.slipping = (tangential_force + angle_force).length() > friction_force;

            force.clear();
            force.apply_force_at_point(
                normal_force + (tangential_force + angle_force).clamp_length_max(friction_force),
                contact_point,
                Vec3::ZERO,
            );
            torque.clear();
            torque.apply_torque(angular_spring_torque - contact_point.cross(angle_force));
        } else {
            debug.grounded = false;
            move_state.neg_cast_vec = capsule_up.try_normalize().unwrap_or(Vec3::Y);
            move_state.contact_point = None;
            force.clear();
            torque.clear();
        }
    }
}

#[derive(SystemParam)]
struct ProceduralRig<'w, 's> {
    query: Query<
        'w,
        's,
        (
            &'static Position,
            &'static Rotation,
            &'static mut ProceduralRigState,
            &'static PlayerMoveState,
            &'static Mass,
            &'static LinearVelocity,
        ),
        With<Player>,
    >,
}

impl<'w, 's> ProceduralRig<'w, 's> {}

fn update_procedural_steps(
    mut query: Query<
        (
            &Position,
            &Rotation,
            &mut ProceduralRigState,
            &PlayerMoveState,
            &Mass,
            &LinearVelocity,
        ),
        With<Player>,
    >,
    _spatial_query: SpatialQuery,
    dt_physics: Res<Time<Physics>>,
    dt_real: Res<Time>,
) {
    let dt = dt_real.delta_seconds() * dt_physics.relative_speed();
    for (
        Position(position),
        Rotation(quat),
        mut rig_state,
        move_state,
        mass,
        LinearVelocity(velocity),
    ) in query.iter_mut()
    {
        let up_dir = *quat * Vec3::Y;
        let right_dir = *quat * Vec3::X;
        let _hip_pos = *position - up_dir * CAPSULE_HEIGHT * 0.5;
        if let Some(contact_point) = move_state.contact_point {
            let contact = contact_point + *position;
            let normal = move_state.contact_normal.unwrap();

            let right_tangent = (right_dir - right_dir.dot(normal) * normal).normalize_or_zero();
            let tangential_vel = *velocity - velocity.dot(normal) * normal;
            let acceleration = (move_state.current_force + move_state.ext_force) / mass.0;
            let lock_duration = 0.06;
            let travel_duration = lock_duration * 2.0;
            let window_pos_ahead = contact
                + 0.5 * (acceleration * 0.5 * lock_duration + tangential_vel) * lock_duration;
            let window_pos_behind = contact
                + 0.5 * (acceleration * 0.5 * lock_duration - tangential_vel) * lock_duration;
            let min_step_size = CAPSULE_RADIUS * 0.5;
            let window_travel_dist = (window_pos_ahead - window_pos_behind).length();
            let ahead_to_contact = (window_pos_ahead - contact).length();
            let slip_vel = if move_state.slipping {
                -0.5 * move_state.current_force / mass.0
            } else {
                Vec3::ZERO
            };
            // println!("{:?}", slip_vel);

            let (i_unlock, i_lock) = match (&rig_state.foot_states[0], &rig_state.foot_states[1]) {
                (FootState::Locked(pos0), FootState::Locked(pos1)) => {
                    if (window_pos_ahead - *pos0).length() > (window_pos_ahead - *pos1).length() {
                        (0, 1)
                    } else {
                        (1, 0)
                    }
                }
                (FootState::Locked(_), FootState::Unlocked(_)) => (0, 1),
                (FootState::Unlocked(_), FootState::Locked(_)) => (1, 0),
                (FootState::Unlocked(info0), FootState::Unlocked(info1)) => {
                    match (info0.target, info1.target) {
                        (Some(_), None) => (1, 0),
                        (None, Some(_)) => (0, 1),
                        (Some(target0), Some(target1)) => {
                            if (info0.pos - target0).length() > (info1.pos - target1).length() {
                                (0, 1)
                            } else {
                                (1, 0)
                            }
                        }
                        _ => (0, 1),
                    }
                }
            };

            let offset_length = 0.3 * CAPSULE_RADIUS;

            for (i, state) in rig_state.foot_states.iter_mut().enumerate() {
                let lr = if i == 0 { -1.0 } else { 1.0 };
                let foot_offset = right_tangent * lr * offset_length;
                match state {
                    FootState::Locked(pos) => {
                        *pos += slip_vel * dt;
                        let next_lock_pos = contact
                            + (acceleration * (travel_duration + 0.5 * lock_duration)
                                + tangential_vel)
                                * (travel_duration + 0.5 * lock_duration)
                            + foot_offset;
                        let _local_travel_dist = (window_pos_ahead - *pos).length();
                        let pos_to_next = (*pos - next_lock_pos).length();
                        let pos_to_contact = (*pos - contact).length();
                        if pos_to_next > min_step_size
                            && pos_to_contact > window_travel_dist * 0.5
                            && pos_to_contact > 1.3 * ahead_to_contact
                        {
                            *state = FootState::Unlocked(FootTravelInfo::unlocked(
                                *pos,
                                travel_duration,
                            ));
                        }
                    }
                    FootState::Unlocked(info) => {
                        if i != i_lock {
                            info.duration += dt;
                        }
                        info.time += dt;
                        if info.time > info.duration {
                            *state = FootState::Locked(window_pos_ahead + foot_offset);
                        } else {
                            let t = info.time / info.duration;
                            let lock_time = info.duration - info.time + 0.5 * lock_duration;
                            let target = contact
                                + (acceleration * lock_time + tangential_vel) * lock_time
                                + foot_offset;
                            info.target = Some(target);
                            let floor_pos = info.pos0.lerp(target, smoothstep(t));
                            let lift = up_dir
                                * 0.1
                                * (target - info.pos0).length()
                                * smoothstep(smoothstep(1.0 - 2.0 * (t - 0.5).abs()));
                            info.pos = floor_pos + lift;
                        }
                    }
                }
            }
        }
    }
}

fn smoothstep(t: f32) -> f32 {
    3.0 * t.powi(2) - 2.0 * t.powi(3)
}

fn set_visible<const VAL: bool>(mut query: Query<&mut Visibility, With<Player>>) {
    for mut visibility in query.iter_mut() {
        if VAL {
            *visibility = Visibility::Visible;
        } else {
            *visibility = Visibility::Hidden;
        }
    }
}

fn draw_debug_gizmos(
    mut query: Query<
        (
            &PhysicsDebugInfo,
            &Position,
            &Rotation,
            &PlayerMoveState,
            &ProceduralRigState,
        ),
        With<Player>,
    >,
    mut gizmos: Gizmos,
    debug_state: Res<State<DebugState>>,
) {
    for (debug, Position(position), Rotation(quat), move_state, steps) in query.iter_mut() {
        if debug.grounded {
            for (i, state) in steps.foot_states.iter().enumerate() {
                let color = if i == 0 {
                    Color::from(RED)
                } else {
                    Color::from(GREEN)
                };
                match state {
                    FootState::Locked(pos) => {
                        gizmos.sphere(*pos, Quat::IDENTITY, 0.5 * CAPSULE_RADIUS, color);
                    }
                    FootState::Unlocked(info) => {
                        gizmos.sphere(
                            info.pos,
                            Quat::IDENTITY,
                            0.5 * CAPSULE_RADIUS,
                            color.with_luminance(0.2),
                        );
                    }
                }
            }

            if debug_state.get() == &DebugState::Torque {
                gizmos
                    .primitive_3d(
                        &Capsule3d::new(CAPSULE_RADIUS, CAPSULE_HEIGHT - CAPSULE_RADIUS * 2.0),
                        position.clone(),
                        debug.delta_quat,
                        Color::from(GRAY),
                    )
                    .resolution(12);
                gizmos.arrow(
                    *position,
                    *position + debug.delta_quat * Vec3::NEG_Z,
                    Color::WHITE,
                );
                let delta_quat = debug.delta_quat * quat.inverse();
                gizmos.arrow(
                    *position,
                    *position + delta_quat.to_scaled_axis(),
                    Color::BLACK,
                );
            }

            if debug_state.get() == &DebugState::Forces {
                let contact_color = if move_state.slipping {
                    Color::from(RED)
                } else {
                    Color::from(GREEN)
                };
                gizmos.sphere(
                    position.clone() + debug.shape_toi * debug.cast_dir,
                    Quat::IDENTITY,
                    CAST_RADIUS,
                    contact_color,
                );
                gizmos.arrow(
                    position.clone() + debug.contact_point,
                    position.clone() + debug.contact_point + debug.spring_force,
                    Color::from(CYAN_100),
                );
                gizmos.arrow(
                    position.clone() + debug.contact_point,
                    position.clone() + debug.contact_point + debug.normal_force,
                    Color::from(BLUE),
                );

                gizmos.arrow(
                    position.clone() + debug.contact_point,
                    position.clone() + debug.contact_point + debug.tangential_force,
                    Color::from(PINK),
                );

                gizmos.arrow(
                    position.clone() + debug.contact_point,
                    position.clone() + debug.contact_point + debug.tangent_vel,
                    Color::from(ORANGE),
                );
                gizmos.arrow(
                    position.clone() + debug.contact_point,
                    position.clone() + debug.contact_point + debug.target_vel,
                    Color::from(GREEN),
                );
                gizmos.arrow(
                    position.clone() + debug.contact_point,
                    position.clone() + debug.contact_point + debug.target_force,
                    Color::from(RED),
                );
                gizmos.arrow(
                    position.clone() + debug.contact_point + debug.tangential_force,
                    position.clone()
                        + debug.contact_point
                        + debug.tangential_force
                        + debug.angle_force,
                    Color::from(YELLOW),
                );
                gizmos.arrow(
                    position.clone(),
                    position.clone() + debug.spring_torque,
                    Color::from(YELLOW),
                );
            }
        }
        gizmos
            .primitive_3d(
                &Capsule3d::new(CAPSULE_RADIUS, CAPSULE_HEIGHT - CAPSULE_RADIUS * 2.0),
                position.clone(),
                quat.clone(),
                Color::WHITE,
            )
            .resolution(12);
        gizmos.arrow(
            *position,
            *position + 0.2 * move_state.forward_dir,
            Color::from(BLUE),
        );
        gizmos.arrow(
            *position,
            *position - 0.2 * move_state.forward_dir,
            Color::from(RED),
        );
    }
}

fn add_results_in_length(dir: Vec3, rhs: Vec3, combined_length: f32) -> Option<Vec3> {
    let dot = dir.dot(rhs);
    let discriminant = dot.powi(2) - rhs.dot(rhs) + combined_length.powi(2);
    if discriminant < 0.0 {
        return None;
    }
    Some((-dot + discriminant.sqrt()) * dir)
}

pub struct PlayerPlugin;

impl Plugin for PlayerPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(InputManagerPlugin::<PlayerAction>::default());
        app.register_type::<PlayerSpawn>();
        app.register_type::<DistanceJoint>();
        app.register_type::<PlayerGroundSpring>();
        app.register_type::<PlayerAngularSpring>();
        app.insert_state(DebugState::None);
        app.insert_resource(SubstepCount(12));
        app.insert_resource(PlayerGroundSpring {
            rest_length: 0.0,
            stiffness: 0.0,
            min_damping: 0.0,
            max_damping: 0.0,
            max_force: 20.0 * 10.0,
            min_force: 0.0,
            smoothing_spring: SpringValue {
                f: 60.0,
                zeta: 10.0,
                m: 1.0,
                velocity: 0.0,
                value: 0.0,
            },
        });
        app.insert_resource(PlayerAngularSpring {
            stiffness: 0.,
            damping: 0.0,
            turn_stiffness: 0.0,
        });
        app.add_systems(Startup, (spawn_camera_3rd_person, spawn_camera_1st_person));
        app.add_systems(
            Update,
            (
                spawn_player,
                player_controls,
                respawn_player,
                toggle_active_view,
                draw_debug_gizmos.run_if(not(in_state(DebugState::None))),
                update_procedural_steps.after(PhysicsSet::Sync),
                (track_camera_3rd_person, track_camera_1st_person)
                    .chain()
                    .after(PhysicsSet::Sync)
                    .run_if(not(in_state(DebugState::None))),
            ),
        );
        app.add_systems(
            SubstepSchedule,
            ((
                (track_camera_3rd_person, track_camera_1st_person)
                    .chain()
                    .run_if(in_state(DebugState::None)),
                update_ground_force,
            )
                .chain())
            .before(IntegrationSet::Velocity),
        );
        app.add_systems(OnEnter(DebugState::None), set_visible::<true>);
        app.add_systems(OnExit(DebugState::None), set_visible::<false>);
        // app.add_plugins(PhysicsDebugPlugin::default());
    }
}

// tests for quaternions

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_delta() {
        let from = Quat::from_mat3(&Mat3::from_cols(Vec3::X, Vec3::Y, Vec3::Z));
        let target = Quat::from_mat3(&Mat3::from_cols(Vec3::NEG_X, Vec3::NEG_Y, Vec3::Z));
        let delta = target * from.inverse();
        println!("{:?}", delta.to_scaled_axis());
        assert!(false);
    }
}
