#version 330 core

struct PBRMaterial {
    sampler2D texAlbedo;
    sampler2D texMetallic;
    sampler2D texRoughness;
    sampler2D texAO;
};

struct PBRMatParams {
    vec3 albedo;
    float metallic;
    float roughness;
    float ao;
};

struct PBRDirLight {
    vec3 direction;
    vec3 color;

    bool enabled;
};

struct PBRPointLight {
    vec3 position;
    vec3 color;

    bool enabled;
};

struct PBRSpotLight {
    vec3 position;
    vec3 direction;
    vec3 color;

    float cutOff;
    float outerCutOff;

    bool enabled;
};

out vec4 fragColor;

in VS_OUT {
    vec3 fragPos;
    vec3 normal;
    vec2 texCoord;
    vec4 fragPos_DirLightSpace;
} fs_in;

uniform vec3 viewPos;

uniform PBRMaterial mat;

const float PI = 3.14159265359;

#define NR_POINT_LIGHTS 16
#define NR_SPOT_LIGHTS 8

uniform PBRDirLight dirLight;
uniform PBRPointLight pointLights[NR_POINT_LIGHTS];
uniform PBRSpotLight spotLights[NR_SPOT_LIGHTS];

/*
layout (std140, binding=0) uniform DirLightBlock {
    PBRDirLight dirLight;
};

layout (std140, binding=1) uniform PointLightBlock {
    PBRPointLight pointLights[NR_POINT_LIGHTS];
};

layout (std140, binding=2) uniform SpotLightBlock {
    PBRSpotLight spotLights[NR_SPOT_LIGHTS];
};
*/

uniform sampler2D shadowMap;

float shadowCalculation(vec4 fragPosLightSpace) {
    float bias = 0.0005;
    vec3 projCoords = fragPosLightSpace.xyz / fragPosLightSpace.w;
    projCoords = projCoords * 0.5 + 0.5;
    float currentDepth = projCoords.z;
    float shadow = 0.0;
    vec2 texelSize = 1.0 / textureSize(shadowMap, 0);
    for (int x = -1; x <= 1; ++x) {
        for (int y = -1; y <= 1; ++y) {
            float pcfDepth = texture(shadowMap, projCoords.xy + vec2(x, y) * texelSize).r;
            shadow += currentDepth - bias > pcfDepth ? 1.0 : 0.0;
        }
    }
    shadow /= 9.0;
    return shadow;
}


vec3 fresnelSchlinck(float cosTheta, vec3 F0) {
    return F0 + (1.0 - F0) * pow(1.0 - cosTheta, 5.0);
}

float distributionGGX(vec3 N, vec3 H, float roughness) {
    float a = roughness * roughness;
    float a2 = a * a;
    float NdotH = max(dot(N, H), 0.0);
    float NdotH2 = NdotH * NdotH;

    float num = a2;
    float denom = (NdotH2 * (a2 - 1.0) + 1.0);
    denom = PI * denom * denom;

    return num / denom;
}

float geometrySchlickGGX(float NdotV, float roughness) {
    float r = (roughness + 1.0);
    float k = (r*r) / 8.0;

    float num = NdotV;
    float denom = NdotV * (1.0 - k) + k;

    return num / denom;
}

float geometrySmith(vec3 N, vec3 V, vec3 L, float roughness) {
    float NdotV = max(dot(N, V), 0.0);
    float NdotL = max(dot(N, L), 0.0);
    float ggx2 = geometrySchlickGGX(NdotV, roughness);
    float ggx1 = geometrySchlickGGX(NdotL, roughness);

    return ggx1 * ggx2;
}

vec3 calcRadiance(vec3 N, vec3 V, vec3 L, vec3 F0, vec3 radiance, PBRMatParams params) {
    vec3 H = normalize(V + L);

    float NDF = distributionGGX(N, H, params.roughness);
    float G = geometrySmith(N, V, L, params.roughness);
    vec3 F = fresnelSchlinck(max(dot(H, V), 0.0), F0);

    vec3 kS = F;
    vec3 kD = vec3(1.0) - kS;

    vec3 numerator = NDF * G * F;
    float denominator = 4.0 * max(dot(N, V), 0.0) * max(dot(N, L), 0.0);
    vec3 specular = numerator / max(denominator, 0.001);

    kD *= 1.0 - params.metallic;

    float NdotL = max(dot(N, L), 0.0);
    return (kD * params.albedo / PI + specular) * radiance * NdotL;
}

vec3 calcDirLight(PBRDirLight light, vec3 N, vec3 V, vec3 F0, PBRMatParams params) {
    vec3 L = normalize(-light.direction);
    vec3 radiance = light.color;

    return calcRadiance(N, V, L, F0, radiance, params);
}

vec3 calcPointLight(PBRPointLight light, vec3 N, vec3 V, vec3 F0, PBRMatParams params) {
    vec3 L = normalize(light.position - fs_in.fragPos);
    float distance = length(light.position - fs_in.fragPos);
    float attenuation = 1.0 / (distance * distance);
    vec3 radiance = light.color * attenuation;

    return calcRadiance(N, V, L, F0, radiance, params);
}

vec3 calcSpotLight(PBRSpotLight light, vec3 N, vec3 V, vec3 F0, PBRMatParams params) {
    vec3 L = normalize(light.position - fs_in.fragPos);
    float distance = length(light.position - fs_in.fragPos);
    float attenuation = 1.0 / (distance * distance);
    vec3 radiance = light.color * attenuation;

    float theta = dot(L, normalize(-light.direction));
    float epsilon = light.cutOff - light.outerCutOff;
    float intensity = clamp((theta - light.outerCutOff) / epsilon, 0.0, 1.0);

    return intensity * calcRadiance(N, V, L, F0, radiance, params);
}

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
    float shadow = shadowCalculation(fs_in.fragPos_DirLightSpace);
    vec3 color = ambient + (1 - shadow) * Lo;

    color = color / (color + vec3(1.0));
    color = pow(color, vec3(1.0 / 2.2));

    fragColor = vec4(color, 1.0);
}

