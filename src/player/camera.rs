use std::f32::consts::PI;

use avian3d::prelude::*;
use bevy::{core_pipeline::motion_blur::MotionBlur, prelude::*};
use leafwing_input_manager::prelude::*;

use super::animation::ProceduralRigState;

#[derive(Component, Reflect, Debug, Default)]
pub struct CameraAnchor3rdPerson {
    pub yaw: f32,
    pub pitch: f32,
}

#[derive(Component, Reflect, Debug, Default)]
pub struct CameraAnchor1stPerson {
    pub yaw: f32,
    pub pitch: f32,
}

pub fn spawn_camera_3rd_person(mut commands: Commands) {
    let camera_arm = 0.22 * Vec3::new(0.0, 4.0, 12.0);
    let mut look_dir = -camera_arm;
    look_dir.x = 0.0;
    let transform =
        Transform::from_translation(camera_arm).looking_to(look_dir.normalize(), Vec3::Y);
    commands
        .spawn((
            Transform::default(),
            Name::new("CameraAnchor"),
            CameraAnchor3rdPerson::default(),
        ))
        .with_children(|builder| {
            builder
                .spawn((
                    Camera3d::default(),
                    Projection::Perspective(PerspectiveProjection {
                        fov: PI / 3.0,
                        ..Default::default()
                    }),
                    Camera {
                        is_active: true,
                        ..Default::default()
                    },
                    transform,
                    MotionBlur {
                        shutter_angle: 1.0,
                        ..Default::default()
                    },
                ))
                .insert(Name::new("PlayerCamera"));
        });
}

pub fn spawn_camera_1st_person(mut commands: Commands) {
    let camera_arm = Vec3::new(0.0, 0.0, 0.0);
    let transform = Transform::from_translation(camera_arm);
    commands
        .spawn((
            Transform::default(),
            Name::new("CameraAnchor"),
            CameraAnchor1stPerson::default(),
        ))
        .with_children(|builder| {
            builder
                .spawn((
                    Camera3d::default(),
                    Camera {
                        is_active: false,
                        ..Default::default()
                    },
                    Projection::Perspective(PerspectiveProjection {
                        fov: PI / 3.0,
                        ..Default::default()
                    }),
                    transform,
                    MotionBlur {
                        shutter_angle: 1.0,
                        ..Default::default()
                    },
                ))
                .insert(Name::new("PlayerCamera"));
        });
}

pub fn toggle_active_view(
    anchor1: Query<(&CameraAnchor1stPerson, &Children)>,
    anchor3: Query<(&CameraAnchor3rdPerson, &Children)>,
    mut cams: Query<&mut Camera>,
    input_query: Query<&ActionState<super::PlayerAction>>,
) {
    for input in input_query.iter() {
        if input.just_pressed(&super::PlayerAction::ViewMode) {
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

pub fn track_camera_3rd_person(
    query: Query<&Position, With<super::Player>>,
    mut camera_query: Query<(&mut Transform, &CameraAnchor3rdPerson)>,
) {
    for Position(pos) in query.iter() {
        for (mut transform, anchor3) in camera_query.iter_mut() {
            transform.translation = pos.clone();
            transform.rotation = Quat::from_euler(EulerRot::YXZ, anchor3.yaw, anchor3.pitch, 0.0);
        }
    }
}

pub fn track_camera_1st_person(
    query: Query<(&Position, &Rotation, &ProceduralRigState), With<super::Player>>,
    mut camera_query: Query<(&mut Transform, &CameraAnchor1stPerson)>,
) {
    for (Position(pos), Rotation(quat), rig_state) in query.iter() {
        let up_dir = *quat * Vec3::Y;

        let cam_up = Quat::IDENTITY.slerp(Quat::from_rotation_arc(Vec3::Y, up_dir), 0.2) * Vec3::Y;
        let pos = rig_state.neck_pos + up_dir * super::physics::CAPSULE_HEIGHT * 0.1;
        for (mut transform, cam1) in camera_query.iter_mut() {
            let view_unrolled = Quat::from_euler(EulerRot::YXZ, cam1.yaw, cam1.pitch, 0.0);
            let forward = view_unrolled * Vec3::NEG_Z;
            *transform = transform.with_translation(pos).looking_to(forward, cam_up);
        }
    }
}
