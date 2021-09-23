//
// Created by lasagnaphil on 21. 9. 23..
//

#ifndef EOS_SCAN_TO_HUMAN_COLLISION_MANAGER_H
#define EOS_SCAN_TO_HUMAN_COLLISION_MANAGER_H

#include <artsim/core/arena.h>
#include <artsim/collision/collision_shape.h>

namespace artsim {

class CollisionManager {
    Arena<CollisionMesh> meshes;
    Arena<CollisionShape> shapes;

    CollisionShape make_ground_shape();
    CollisionShape make_box_shape(glm::vec3 size);
    CollisionShape make_sphere_shape(real radius);
    CollisionShape make_mesh_shape_bvh(const char* objfile, glm::rvec3 scale = glm::rvec3(1));
    CollisionShape make_mesh_shape_sdf(const char* objfile, real cell_size, glm::rvec3 scale = glm::rvec3(1));
};

}
#endif //EOS_SCAN_TO_HUMAN_COLLISION_MANAGER_H
