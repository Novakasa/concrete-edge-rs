use bevy::prelude::*;

#[derive(Debug, Clone, Reflect, Component, Default)]
pub enum RigSide {
    #[default]
    Left,
    Right,
}

impl RigSide {
    fn opposite(&self) -> Self {
        match self {
            Self::Left => Self::Right,
            Self::Right => Self::Left,
        }
    }
}

#[derive(Debug, Clone, Default, Reflect)]
pub struct UnlockedFootInfo {
    pub pos: Vec3,
    pos0: Vec3,
    target: Vec3,
}

#[derive(Debug, Clone, Reflect)]
pub struct LockedFootInfo {
    pub pos: Vec3,
}

#[derive(Debug, Reflect, Clone)]
pub enum FootState {
    Locked(LockedFootInfo),
    Unlocked(UnlockedFootInfo),
}

impl FootState {
    fn locked(pos: Vec3) -> Self {
        Self::Locked(LockedFootInfo { pos: pos })
    }

    fn unlocked(pos: Vec3, target: Vec3) -> Self {
        Self::Unlocked(UnlockedFootInfo {
            pos: pos,
            pos0: pos,
            target: target,
        })
    }

    fn is_unlocked(&self) -> bool {
        matches!(self, Self::Unlocked(_))
    }
}

impl Default for FootState {
    fn default() -> Self {
        Self::Unlocked(UnlockedFootInfo::default())
    }
}

#[derive(Debug, Clone, Default, Component)]
struct RunCycle {
    pub t: f32,
    pub last_lock: RigSide,
    pub t_unlock: f32,
}

#[derive(Debug, Clone, Default, Component)]
struct RigState {
    pub cm_offset: Vec3,
    pub cm_offset_vel: Vec3,
    pub hip_pos: Vec3,
    pub neck_pos: Vec3,
    pub shoulder_pos: Vec3,
    pub head_pos: Vec3,
}

struct AnimationPlugin;

impl Plugin for AnimationPlugin {
    fn build(&self, app: &mut App) {}
}
