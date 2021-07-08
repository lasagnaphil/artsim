#version 430 core

#include "pbr.frag"

layout (location = 0) out vec4 frag;

void main() {
    vec3 N = normalize(fs_in.normal);
    vec3 V = normalize(viewPos - fs_in.fragPos);

    PBRMatParams params;
    params.albedo = mat.albedo * texture(mat.texAlbedo, fs_in.texCoord).rgb;
    params.metallic = mat.metallic * texture(mat.texMetallic, fs_in.texCoord).r;
    params.roughness = mat.roughness * texture(mat.texRoughness, fs_in.texCoord).r;
    params.ao = mat.ao * texture(mat.texAO, fs_in.texCoord).r;

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
    float shadow = shadowCalculation(fs_in.fragPos_DirLightSpace);
    vec3 color = ambient + (1 - shadow) * Lo;

    frag = vec4(color, 1.0);
}
