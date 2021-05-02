#version 430 core

#include "pbr.frag"

layout (location = 0) out vec4 accum;
layout (location = 1) out float reveal;

void main() {
    vec3 N = normalize(fs_in.normal);
    vec3 V = normalize(viewPos - fs_in.fragPos);

    PBRMatParams params;
    params.albedo = texture(mat.texAlbedo, fs_in.texCoord).rgb;
    params.metallic = texture(mat.texMetallic, fs_in.texCoord).r;
    params.roughness = texture(mat.texRoughness, fs_in.texCoord).r;
    params.ao = texture(mat.texAO, fs_in.texCoord).r;

    vec3 F0 = vec3(0.04);
    F0 = mix(F0, params.albedo, params.metallic);

    vec3 Lo = vec3(0.0);
    if (dirLight.enabled) {
        Lo = calcDirLight(dirLight, N, V, F0, params);
    }

    for (int i = 0; i < NR_POINT_LIGHTS; ++i) {
        if (pointLights[i].enabled) {
            Lo += calcPointLight(pointLights[i], N, V, F0, params);
        }
    }

    for (int i = 0; i < NR_SPOT_LIGHTS; ++i) {
        if (spotLights[i].enabled) {
            Lo += calcSpotLight(spotLights[i], N, V, F0, params);
        }
    }

    vec3 ambient = vec3(0.03) * params.albedo * params.ao;
    vec3 color = ambient + Lo;

    float alpha = mat.alpha;

    // weight function
    float weight = clamp(pow(min(1.0, alpha * 10.0) + 0.01, 3.0) * 1e8 * pow(1.0 - gl_FragCoord.z * 0.9, 3.0), 1e-2, 3e3);

    // store pixel color accumulation
    accum = vec4(color * alpha, alpha) * weight;

    // store pixel revealage threshold
    reveal = alpha;
}
