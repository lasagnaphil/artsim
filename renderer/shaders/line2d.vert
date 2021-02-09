#version 330 core

layout (location = 0) in vec2 inPos;

uniform float height;
uniform float pointSize;

uniform mat4 model;
uniform mat4 view;
uniform mat4 proj;

void main() {
    gl_Position = proj * view * model * vec4(inPos.x, height, inPos.y, 1.0f);
    gl_PointSize = pointSize;
}