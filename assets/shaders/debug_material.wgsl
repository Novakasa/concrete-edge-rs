#import bevy_pbr::{pbr_fragment::pbr_input_from_standard_material}

#ifdef PREPASS_PIPELINE
#import bevy_pbr::{
    prepass_io::{VertexOutput, FragmentOutput},
    pbr_deferred_functions::deferred_output,
}
#else
#import bevy_pbr::{forward_io::{VertexOutput, FragmentOutput}, pbr_functions::{apply_pbr_lighting, main_pass_post_lighting_processing},}
#endif


@fragment
fn fragment(
    mesh: VertexOutput,
    @builtin(front_facing) is_front: bool,
) -> FragmentOutput {
    var pbr_input = pbr_input_from_standard_material(mesh, is_front);

    let coord = abs(mesh.world_position * 1.0 + 1.0e5);
    let modifier = floor(coord);
    let sum = (modifier.x + modifier.z + 0.0 * modifier.y) % 2.0;


    var out: FragmentOutput;
    out.color = apply_pbr_lighting(pbr_input);
    out.color = out.color * (1.0 - 0.2 * sum);
    out.color = main_pass_post_lighting_processing(pbr_input, out.color);
    return out;
    // return vec4<f32>(sum, sum, sum, 1.0);
}