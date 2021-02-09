#version 330 core

layout (location = 0) in vec3 inPos;
layout (location = 1) in vec3 inNormal;
layout (location = 2) in vec2 inTexCoord;

uniform mat4 model;
uniform mat4 view;
uniform mat4 proj;

out vec3 worldPos;
out vec3 normal;
out vec2 texCoord;

void main()
{
    gl_Position = proj * view * model * vec4(inPos, 1.0);
    worldPos = vec3(model * vec4(inPos, 1.0));
    normal = mat3(transpose(inverse(model))) * inNormal;
    texCoord = inTexCoord;
}
