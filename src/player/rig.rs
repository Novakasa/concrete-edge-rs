use bevy::{color::palettes::css::BLACK, prelude::*};

use super::animation::ProceduralRigState;

const STICK_RADIUS: f32 = 0.1;

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
    pub fn scale() -> f32 {
        0.0080
    }

    pub fn length(&self) -> f32 {
        let raw = match self {
            Self::LowerBack => 40.0,
            Self::UpperBack => 40.0,
            Self::LeftUpperLeg | Self::RightUpperLeg => 44.0,
            Self::LeftLowerLeg | Self::RightLowerLeg => 44.0,
            Self::Head => 50.0,
        };
        raw * Self::scale()
    }

    pub fn height() -> f32 {
        RigBone::LeftUpperLeg.length()
            + RigBone::LeftLowerLeg.length()
            + RigBone::LowerBack.length()
            + RigBone::UpperBack.length()
            + RigBone::Head.length()
    }

    pub fn leg_length() -> f32 {
        RigBone::LeftUpperLeg.length() + RigBone::LeftLowerLeg.length()
    }

    pub fn legacy_capsule_height() -> f32 {
        let back_length = RigBone::LowerBack.length() + RigBone::UpperBack.length();
        let legacy_back_length = 65.0 * Self::scale();
        back_length / legacy_back_length * 0.8
    }

    pub fn legacy_capsule_radius() -> f32 {
        let back_length = RigBone::LowerBack.length() + RigBone::UpperBack.length();
        let legacy_back_length = (40.0 + 40.0) * Self::scale();
        back_length / legacy_back_length * 0.2
    }

    pub fn max_contact_dist() -> f32 {
        Self::legacy_capsule_height() * 0.9 + 0.16
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
    println!("Height: {}", RigBone::height());
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
