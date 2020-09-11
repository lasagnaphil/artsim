//
// Created by lasagnaphil on 20. 5. 19..
//

#include "artsim/artsim.h"
#include "artsim/math/se3.h"

using namespace artsim;

float Shape::mass(float density) {
    switch (type) {
        case Type::Box: {
            return density * box.size.x * box.size.y * box.size.z;
        } break;
        case Type::Sphere: {
            return 4.f / 3.f * glm::pi<float>() * sphere.radius * sphere.radius * sphere.radius;
        } break;
    }
}

glm::mat3 Shape::inertia(float density) {
    glm::vec3 I;
    switch (type) {
        case Type::Box: {
            const glm::vec3& s = box.size;
            I = mass(density) * glm::vec3(s.y*s.y + s.z*s.z, s.z*s.z + s.x*s.x, s.x*s.x + s.y*s.y) / 12.f;
        } break;
        case Type::Sphere: {
            float r = sphere.radius;
            I = 0.4f * mass(density) * glm::vec3(r*r);
        } break;
    }
    return glm::mat3(I.x, 0, 0, 0, I.y, 0, I.z, 0, 0);
}
