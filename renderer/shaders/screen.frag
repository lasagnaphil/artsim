#version 400 core

// shader inputs
in vec2 texture_coords;

// shader outputs
layout (location = 0) out vec4 frag;

// screen image
uniform sampler2D screen;

uniform float exposure;

void main()
{
    vec3 hdrColor = texture(screen, texture_coords).rgb;
    vec3 mapped = vec3(1.0) - exp(-hdrColor * exposure);
    vec3 color = pow(mapped, vec3(1.0 / 2.2));
    frag = vec4(color, 1.0f);
}