#version 330 core

layout (location = 0) in vec2 inPos;
layout (location = 1) in vec3 inColor;
layout (location = 2) in float inSize;

out vec3 color;

uniform mat4 view;
uniform mat4 proj;

void main() {
    gl_Position = proj * view * vec4(inPos, 0.0, 1.0);
    gl_PointSize = inSize;
    color = inColor;
}