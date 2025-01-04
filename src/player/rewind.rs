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
}

#[derive(Component, Reflect, Debug, Default)]
pub struct RewindHistory(pub Vec<HistoryState>);

impl RewindHistory {
    fn get_state_at_time(&self, rewind_time: &f32) -> &HistoryState {
        let index = self
            .0
            .len()
            .checked_sub((-rewind_time + 1.0) as usize)
            .unwrap_or(0);
        &self.0[index]
    }

    fn discard_after_time(&mut self, rewind_time: f32) {
        let index = self
            .0
            .len()
            .checked_sub((-rewind_time + 1.0) as usize)
            .unwrap_or(0);
        self.0.truncate(index);
    }
}

#[derive(Component, Reflect, Debug, Default, Clone)]
pub struct HistoryState {
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
            position: *position,
            velocity: *velocity,
            rotation: *rotation,
            angular_velocity: *angular_velocity,
            physics_state: physics_state.clone(),
        });
        if history.0.len() > 300 {
            history.0.remove(0);
        }
    }
}

fn init_rewind(mut rewind_info: ResMut<RewindInfo>, mut time: ResMut<Time<Physics>>) {
    rewind_info.rewind_time = 0.0;
    println!("Rewind started");
    time.pause();
}

fn exit_rewind(
    mut rewind_info: ResMut<RewindInfo>,
    mut time: ResMut<Time<Physics>>,
    mut q_historyt: Query<&mut RewindHistory>,
) {
    for mut history in q_historyt.iter_mut() {
        history.discard_after_time(rewind_info.rewind_time);
    }
    time.unpause();
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
        rewind_info.rewind_time = rewind_info
            .rewind_time
            .clamp(-(history.0.len() as f32), 0.0);
        let entry = history.get_state_at_time(&rewind_info.rewind_time);
        position.0 = entry.position;
        rotation.0 = entry.rotation;
        velocity.0 = entry.velocity;
        angular_velocity.0 = entry.angular_velocity;
        *physics_state = entry.physics_state.clone();
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
