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

enum class CollisionFlags {
    CF_DYNAMIC_OBJECT = 0,
    CF_STATIC_OBJECT = 1,
    CF_KINEMATIC_OBJECT = 2,
    CF_DETECT_SELF_COLLISIONS = 4
};

struct World;

struct RigidBody {
    World* world;

    Id<CollisionShape> shape_id;
    Id<Material> mat_id;

    CollisionFlags collision_flags;

    glmx::tbox<3, real> bounds;
    glm::rvec3 bounds_center;

    glmx::rsmat3x3 inertia, inv_inertia;
    real mass, inv_mass;
    glmx::rquat_transform offset_from_com;

    glmx::rquat_transform world_trans;
    glmx::rscrew body_vel;
    glm::rvec3 acc;
    glm::rvec3 angacc;
    glm::rvec3 f_ext;
    glm::rvec3 tau_ext;
    glm::rvec3 f_c;
    glm::rvec3 tau_c;

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
