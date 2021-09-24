//
// Created by lasagnaphil on 21. 9. 23..
//

#ifndef EOS_SCAN_TO_HUMAN_COLLISION_MANAGER_H
#define EOS_SCAN_TO_HUMAN_COLLISION_MANAGER_H

#include <artsim/core/arena.h>
#include <artsim/collision/collision_shape.h>
#include <artsim/collision/collision_object.h>

namespace artsim {

struct ContactManifoldPoint {
    glm::rvec3 pos;
    glm::rvec3 normal;
    glm::rvec3 tangent1;
    glm::rvec3 tangent2;
    glm::rvec3 lam;
    real distance;
    real area;
};

struct ContactManifold {
    Id<RigidBody> body1_id;
    Id<RigidBody> body2_id;
    glmx::rtransform body1_rel_trans;
    glmx::rtransform body2_rel_trans;
    std::vector<Id<ContactManifoldPoint>> points;
};

class CollisionManager {
public:
    using AABB = glmx::tbox<3, real>;

    CollisionShape make_ground_shape();
    CollisionShape make_box_shape(glm::vec3 size);
    CollisionShape make_sphere_shape(real radius);
    CollisionShape make_mesh_shape_bvh(const char* objfile, glm::rvec3 scale = glm::rvec3(1));
    CollisionShape make_mesh_shape_sdf(const char* objfile, real cell_size, glm::rvec3 scale = glm::rvec3(1));

    void init_bvh();

private:
    struct BVHNode {
        AABB aabb;
        int left_id = -1, right_id = -1;
        Id<CollisionObject> obj_id;
    };

    Arena<CollisionMesh> meshes;
    Arena<CollisionShape> shapes;
    Arena<CollisionObject> objects;

    Arena<ContactManifold> contact_manifolds;

    std::vector<BVHNode> nodes;

    void create_children(int node_id, std::vector<Id<CollisionObject>>& queue);

};

}
#endif //EOS_SCAN_TO_HUMAN_COLLISION_MANAGER_H
