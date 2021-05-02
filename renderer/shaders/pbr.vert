#version 430 core

#define NR_POINT_LIGHTS 16
#define NR_SPOT_LIGHTS 8

layout (location = 0) in vec3 inPos;
layout (location = 1) in vec3 inNormal;
layout (location = 2) in vec2 inTexCoord;

uniform mat4 model;
uniform mat4 view;
uniform mat4 proj;
uniform mat4 dirLightSpaceMatrix;

out VS_OUT {
    vec3 fragPos;
    vec3 normal;
    vec2 texCoord;
    vec4 fragPos_DirLightSpace;
} vs_out;

void main()
{
    gl_Position = proj * view * model * vec4(inPos, 1.0);

    vs_out.fragPos = vec3(model * vec4(inPos, 1.0));
    vs_out.texCoord = inTexCoord;
    vs_out.normal = vec3(transpose(inverse(model)) * vec4(inNormal, 1.0));
    vs_out.fragPos_DirLightSpace = dirLightSpaceMatrix * vec4(vs_out.fragPos, 1.0);
}
