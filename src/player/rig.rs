use avian3d::prelude::*;
use bevy::{
    color::palettes::css::{BLACK, LIME, WHITE},
    prelude::*,
};

use crate::util::ik2_positions;

use super::animation::{FootState, ProceduralRigState};

const STICK_RADIUS: f32 = 0.08;

#[derive(Component, Reflect, Debug, Clone, Hash, PartialEq, Eq)]
pub enum RigBone {
    LowerBack,
    UpperBack,
    LeftUpperLeg,
    LeftLowerLeg,
    RightUpperLeg,
    RightLowerLeg,
    Head,
}

impl RigBone {
    pub fn length(&self) -> f32 {
        let raw = match self {
            Self::LowerBack => 40.0,
            Self::UpperBack => 40.0,
            Self::LeftUpperLeg | Self::RightUpperLeg => 44.0,
            Self::LeftLowerLeg | Self::RightLowerLeg => 44.0,
            Self::Head => 50.0,
        };
        raw * 0.0065
    }

    pub fn leg_length() -> f32 {
        RigBone::LeftUpperLeg.length() + RigBone::LeftLowerLeg.length()
    }
}

fn spawn_meshes(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let mut material = StandardMaterial::from_color(Color::from(BLACK));
    material.perceptual_roughness = 0.8;
    let material = materials.add(material);

    for bone in [
        RigBone::LeftUpperLeg,
        RigBone::RightUpperLeg,
        RigBone::LeftLowerLeg,
        RigBone::RightLowerLeg,
        RigBone::LowerBack,
        RigBone::UpperBack,
    ]
    .iter()
    {
        commands.spawn((
            Mesh3d(meshes.add(Mesh::from(Capsule3d::new(STICK_RADIUS, bone.length())))),
            bone.clone(),
            MeshMaterial3d(material.clone()),
        ));
    }
    commands.spawn((
        Mesh3d(meshes.add(Mesh::from(Sphere::new(
            RigBone::Head.length() * 0.5 + STICK_RADIUS * 0.5,
        )))),
        RigBone::Head,
        MeshMaterial3d(material.clone()),
    ));
}

fn update_bones(
    mut bones: Query<(&RigBone, &mut Transform, &mut Visibility)>,
    rig: Query<&ProceduralRigState>,
) {
    for steps in rig.iter() {
        let transforms = steps.get_bone_transforms();
        for (bone, mut transform, mut visivility) in bones.iter_mut() {
            if let Some(new_transform) = transforms.get(bone) {
                *transform = *new_transform;
                *visivility = Visibility::Visible;
            } else {
                *visivility = Visibility::Hidden;
            }
        }
    }
}

pub struct RigPlugin;

impl Plugin for RigPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, spawn_meshes);
        app.add_systems(
            Update,
            update_bones.after(super::animation::update_procedural_state),
        );
    }
}
