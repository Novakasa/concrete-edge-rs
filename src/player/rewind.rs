use super::physics::*;
use avian3d::prelude::*;
use bevy::prelude::*;

#[derive(States, Debug, Clone, Eq, PartialEq, Hash)]
pub enum RewindState {
    Playing,
    Rewinding,
}

#[derive(Resource, Debug, Default)]
pub struct RewindInfo {
    pub rewind_time: f32,
    pub latest_time: f32,
}

#[derive(Component, Reflect, Debug, Default)]
pub struct RewindHistory(pub Vec<HistoryState>);

impl RewindHistory {
    fn get_state_at_time(&self, rewind_time: &f32) -> &HistoryState {
        let index = self.index_at_time(rewind_time);
        &self.0[index]
    }

    fn discard_after_time(&mut self, rewind_time: &f32) {
        let index = self.index_at_time(rewind_time);
        self.0.truncate(index);
    }

    fn index_at_time(&self, rewind_time: &f32) -> usize {
        let mut index = 0;
        for (i, entry) in self.0.iter().enumerate() {
            if entry.time > *rewind_time {
                break;
            }
            index = i;
        }
        index
    }

    fn get_latest_time(&self) -> f32 {
        self.0.last().unwrap().time
    }

    fn get_earliest_time(&self) -> f32 {
        self.0.first().unwrap().time
    }
}

#[derive(Component, Reflect, Debug, Default, Clone)]
pub struct HistoryState {
    pub time: f32,
    pub position: Vec3,
    pub velocity: Vec3,
    pub rotation: Quat,
    pub angular_velocity: Vec3,
    pub physics_state: PhysicsState,
}

fn record_history(
    mut q_physics: Query<(
        &Position,
        &LinearVelocity,
        &Rotation,
        &AngularVelocity,
        &PhysicsState,
        &mut RewindHistory,
    )>,
    mut rewind_info: ResMut<RewindInfo>,
    physics_time: Res<Time<Physics>>,
) {
    for (
        Position(position),
        LinearVelocity(velocity),
        Rotation(rotation),
        AngularVelocity(angular_velocity),
        physics_state,
        mut history,
    ) in q_physics.iter_mut()
    {
        history.0.push(HistoryState {
            time: rewind_info.latest_time,
            position: *position,
            velocity: *velocity,
            rotation: *rotation,
            angular_velocity: *angular_velocity,
            physics_state: physics_state.clone(),
        });
        if history.0.len() > 3000 {
            history.0.remove(0);
        }
    }
    rewind_info.latest_time += physics_time.delta_secs();
}

fn init_rewind(mut rewind_info: ResMut<RewindInfo>, mut time: ResMut<Time<Physics>>) {
    rewind_info.rewind_time = rewind_info.latest_time;
    time.pause();
}

fn exit_rewind(
    mut rewind_info: ResMut<RewindInfo>,
    mut time: ResMut<Time<Physics>>,
    mut q_historyt: Query<&mut RewindHistory>,
) {
    for mut history in q_historyt.iter_mut() {
        history.discard_after_time(&rewind_info.rewind_time);
    }
    time.unpause();
    rewind_info.latest_time = rewind_info.rewind_time;
}

fn update_rewind(
    mut q_physics: Query<(
        &mut Position,
        &mut Rotation,
        &mut LinearVelocity,
        &mut AngularVelocity,
        &mut PhysicsState,
        &RewindHistory,
    )>,
    mut rewind_info: ResMut<RewindInfo>,
) {
    for (
        mut position,
        mut rotation,
        mut velocity,
        mut angular_velocity,
        mut physics_state,
        history,
    ) in q_physics.iter_mut()
    {
        let entry = history.get_state_at_time(&rewind_info.rewind_time);
        position.0 = entry.position;
        rotation.0 = entry.rotation;
        velocity.0 = entry.velocity;
        angular_velocity.0 = entry.angular_velocity;
        *physics_state = entry.physics_state.clone();
        rewind_info.rewind_time = rewind_info
            .rewind_time
            .clamp(history.get_earliest_time(), history.get_latest_time());
    }
}

pub struct RewindPlugin;

impl Plugin for RewindPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(RewindInfo::default());
        app.add_systems(
            FixedPostUpdate,
            record_history
                .before(set_external_force)
                .run_if(in_state(RewindState::Playing)),
        );
        app.add_systems(OnEnter(RewindState::Rewinding), init_rewind);
        app.add_systems(OnExit(RewindState::Rewinding), exit_rewind);
        app.add_systems(
            Update,
            update_rewind.run_if(in_state(RewindState::Rewinding)),
        );
        app.insert_state(RewindState::Playing);
    }
}
