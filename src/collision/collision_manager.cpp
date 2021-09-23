//
// Created by lasagnaphil on 21. 9. 23..
//

#include <artsim/collision/collision_manager.h>

namespace artsim {

CollisionShape CollisionManager::make_ground_shape() {
    CollisionShape shape;
    shape.type = CollisionShape::Type::Ground;
    shape.scale = glm::rvec3(1);
    return shape;
}

CollisionShape CollisionManager::make_box_shape(glm::vec3 size) {
    CollisionShape shape;
    shape.type = CollisionShape::Type::Box;
    shape.scale = size;
    return shape;
}

CollisionShape CollisionManager::make_sphere_shape(real radius) {
    CollisionShape shape;
    shape.type = CollisionShape::Type::Sphere;
    shape.scale = glm::rvec3(radius, radius, radius);
    return shape;
}

CollisionShape CollisionManager::make_mesh_shape_bvh(const char* objfile, glm::rvec3 scale) {
    CollisionShape shape;
    shape.type = CollisionShape::Type::Mesh;
    shape.scale = scale;
    return shape;
}

CollisionShape CollisionManager::make_mesh_shape_sdf(const char* objfile, real cell_size, glm::rvec3 scale) {
    CollisionShape shape;
    shape.type = CollisionShape::Type::Mesh;
    shape.scale = scale;
    return shape;
}

}