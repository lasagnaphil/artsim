//
// Created by lasagnaphil on 21. 9. 23..
//

#ifndef EOS_SCAN_TO_HUMAN_COLLISION_MANAGER_H
#define EOS_SCAN_TO_HUMAN_COLLISION_MANAGER_H

#include <artsim/core/arena.h>
#include <artsim/collision/collision_shape.h>

namespace artsim {

struct World;

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

    CollisionManager(World* world = nullptr) : world(world) {}

    void rebuild();

    void add_rigid_body(Id<RigidBody> rb_id);
    void remove_rigid_body(Id<RigidBody> rb_id);

    void find_collisions();

private:
    struct BVHNode {
        AABB aabb;
        int parent_id = -1;
        int left_id = -1, right_id = -1;
        Id<RigidBody> rb_id;
    };

    World* world;
    std::vector<BVHNode> nodes;

    Arena<ContactManifold> manifolds;
    Arena<ContactManifoldPoint> manifold_points;

    void create_children(int node_id, std::vector<Id<RigidBody>>& queue);
};

}
#endif //EOS_SCAN_TO_HUMAN_COLLISION_MANAGER_H
