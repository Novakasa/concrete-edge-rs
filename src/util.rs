use bevy::prelude::*;

pub fn cosc_from_sides(a: f32, b: f32, c: f32) -> f32 {
    // law of cosines
    (a * a + b * b - c * c) / (2.0 * a * b)
}

pub fn ik2_positions(len1: f32, len2: f32, target: Vec3, bend_dir: Vec3) -> (Vec3, Vec3) {
    let target_dir = target.try_normalize().unwrap();
    let bend_orth = target_dir
        .cross(bend_dir)
        .cross(target_dir)
        .try_normalize()
        .unwrap();
    let min_range = len1.max(len2) - len1.min(len2);
    let max_range = len1 + len2;
    let target_dir = target.normalize();
    if target.length() > max_range {
        return (target_dir * len1, target_dir * (len1 + len2));
    }
    if target.length() < min_range {
        return if len1 > len2 {
            (target_dir * len1, target_dir * min_range)
        } else {
            (-target_dir * len1, target_dir * min_range)
        };
    }
    let cos_a1 = cosc_from_sides(len1, target.length(), len2);
    let sin_a1 = (1.0 - cos_a1 * cos_a1).sqrt();
    let local_pos2 = target_dir * cos_a1 * len1 + bend_orth * sin_a1 * len1;
    (local_pos2, target)
}

pub fn ik3_positions(
    len1: f32,
    len2: f32,
    len3: f32,
    target: Vec3,
    target_dir: Vec3,
    bend_dir: Vec3,
) -> (Vec3, Vec3, Vec3) {
    let segment3 = target_dir * len3;
    let target_segment12 = target - segment3;
    let min_range = len1.max(len2) - len1.min(len2);
    let max_range = len1 + len2;

    if target_segment12.length() > max_range {
        let (pos2, pos3) = ik2_positions(max_range, len3, target, bend_dir);
        let pos1 = pos2.normalize() * len1;
        return (pos1, pos2, pos3);
    }
    if target_segment12.length() < min_range {
        let (pos2, pos3) = ik2_positions(min_range, len3, target, bend_dir);
        let mut pos1 = pos2.normalize() * len1;
        if len1 < len2 {
            pos1 *= -1.0;
        }
        return (pos1, pos2, pos3);
    }
    let (po1, pos2) = ik2_positions(len1, len2, target_segment12, bend_dir);
    let pos3 = target;

    (po1, pos2, pos3)
}
