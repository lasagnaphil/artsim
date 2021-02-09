#version 330 core

#define NR_POINT_LIGHTS 16
#define NR_SPOT_LIGHTS 8

layout (location = 0) in vec3 inPos;
layout (location = 1) in vec3 inNormal;
layout (location = 2) in vec2 inTexCoord;

/*
layout (location = 3) in vec3 inTangent;
layout (location = 4) in vec3 inBitangent;

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

uniform PBRDirLight dirLight;
uniform PBRPointLight pointLights[NR_POINT_LIGHTS];
uniform PBRSpotLight spotLights[NR_SPOT_LIGHTS];

uniform vec3 viewPos;

*/
uniform mat4 model;
uniform mat4 view;
uniform mat4 proj;
uniform mat4 dirLightSpaceMatrix;

out VS_OUT {
    vec3 fragPos;
    vec3 normal;
    vec2 texCoord;
    vec4 fragPos_DirLightSpace;

    /*
    vec3 tangentPointLightPos[NR_POINT_LIGHTS];
    vec3 tangentSpotLightPos[NR_SPOT_LIGHTS];
    vec3 tangentViewPos;
    vec3 tangentFragPos;
    */
} vs_out;

void main()
{
    gl_Position = proj * view * model * vec4(inPos, 1.0);

    vs_out.fragPos = vec3(model * vec4(inPos, 1.0));
    vs_out.texCoord = inTexCoord;
    vs_out.normal = vec3(transpose(inverse(model)) * vec4(inNormal, 1.0));
    vs_out.fragPos_DirLightSpace = dirLightSpaceMatrix * vec4(vs_out.fragPos, 1.0);

    /*
    mat3 normalMatrix = transpose(inverse(mat3(model)));
    vec3 T = normalize(vec3(normalMatrix * inTangent));
    vec3 N = normalize(vec3(normalMatrix * inNormal));
    T = normalize(T - dot(T, N) * N);
    vec3 B = cross(N, T);

    mat3 TBN = transpose(mat3(T, B, N));

    for (int i = 0; i < NR_POINT_LIGHTS; i++) {
        if (pointLights[i].enabled) {
            vs_out.tangentPointLightPos[i] = TBN * pointLights[i].position;
        }
    }
    for (int i = 0; i < NR_SPOT_LIGHTS; i++) {
        if (spotLights[i].enabled) {
            vs_out.tangentSpotLightPos[i] = TBN * spotLights[i].position;
        }
    }

    vs_out.tangentViewPos = TBN * viewPos;
    vs_out.tangentFragPos = TBN * vs_out.fragPos;
    */
}
