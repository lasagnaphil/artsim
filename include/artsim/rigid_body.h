//
// Created by lasagnaphil on 8/21/21.
//

#ifndef ARTSIM_RIGID_BODY_H
#define ARTSIM_RIGID_BODY_H

#include <artsim/math/se3.h>
#include <artsim/math/box.h>
#include <artsim/types.h>
#include <artsim/collision/collision_shape.h>
#include <artsim/material.h>

#include <BulletCollision/BroadphaseCollision/btBroadphaseProxy.h>

class btCollisionShape;
class btCollisionWorld;
class btCollisionObject;

namespace artsim {

enum CollisionFlags : uint32_t {
    CF_DYNAMIC_OBJECT = 0,
    CF_STATIC_OBJECT = 1,
    CF_KINEMATIC_OBJECT = 2,
    CF_DETECT_SELF_COLLISIONS = 4
};

enum CollisionMask : uint32_t {
    CM_DEFAULT = 1,
    CM_STATIC = 2,
    CM_KINEMATIC = 4,
    CM_CHARACTER = 8,
    CM_ALL = 0xffffffff,
};

struct World;

struct RigidBody {
    World* world;

    Id<CollisionShape> shape_id;
    Id<Material> mat_id;

    CollisionFlags collision_flags;
    CollisionMask filter_group;
    CollisionMask filter_mask;

    real mass;
    glmx::rsmat3x3 inertia;
    glmx::rquat_transform offset_from_com;

    glmx::tbox<3, real> bounds;
    glm::rvec3 bounds_center;
    int bvh_id = -1;

    glmx::rsmat6x6 I;

    glmx::rquat_transform world_trans;
    glmx::rscrew body_vel;
    glmx::rscrew body_acc;
    glmx::rscrew body_f_ext;
    glmx::rscrew body_f_c;

    void reset();
    void randomize_positions();

    void update_colliders();
    void forward_dynamics(const rvec3& gravity);
    void integrate(real dt);
    void integrate_positions(real dt);
    void integrate_velocities(real dt);
    void simulate(const rvec3& gravity, real dt);
};

}
#endif //ARTSIM_RIGID_BODY_H
