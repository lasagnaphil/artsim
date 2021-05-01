#include "pbr.frag"

out vec4 fragColor;

void main() {
    vec3 color = lightCalculation();
    fragColor = vec4(color, 1.0);
}
