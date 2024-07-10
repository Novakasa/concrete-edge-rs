#import bevy_pbr::forward_io::VertexOutput
#import bevy_render::view  View

@group(2) @binding(0) var<uniform> material_color: vec4<f32>;

@group(0) @binding(0) var<uniform> view: View;

@fragment
fn fragment(
    mesh: VertexOutput,
) -> @location(0) vec4<f32> {
    let coord = abs(mesh.world_position * 1.0 + 1.0e5);
    let modifier = floor(coord);
    let sum = (modifier.x + modifier.z + 0.0 * modifier.y) % 2.0;
    return material_color * (1.0 - 0.2 * sum);
}