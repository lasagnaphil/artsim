#version 330 core

uniform vec4 color;

out vec4 fragColor;

void main() {
    vec2 circCoord = 2.0 * gl_PointCoord - 1.0;
    if (dot(circCoord, circCoord) > 1.0) {
        discard;
    }
    fragColor = color;
}
