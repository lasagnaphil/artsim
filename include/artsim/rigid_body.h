//
// Created by lasagnaphil on 8/21/21.
//

#ifndef ARTSIM_RIGID_BODY_H
#define ARTSIM_RIGID_BODY_H

#include <artsim/math/se3.h>
#include <artsim/types.h>
#include <artsim/collision_shape.h>
#include <artsim/material.h>

#include <BulletCollision/BroadphaseCollision/btBroadphaseProxy.h>

class btCollisionShape;
class btCollisionWorld;
class btCollisionObject;

namespace artsim {

struct RigidBodySpec {
    CollisionShape col_shape;
    glmx::tsmat3x3<real> inertia, inv_inertia;
    real mass, inv_mass;

    RigidBodySpec() = default;
    RigidBodySpec(const CollisionShape& shape, real density)
        : RigidBodySpec(shape, shape.mass(density), shape.inertia(density)) {
    }
    RigidBodySpec(const CollisionShape& shape, real mass, glmx::rsmat3x3 inertia)
        : col_shape(shape), mass(mass), inertia(inertia) {
        inv_inertia = glmx::inverse(inertia);
        inv_mass = real(1) / mass;
    }
};

struct RigidBody {
    RigidBodySpec spec;
    glm::rvec3 pos;
    glm::rquat rot;
    glm::rvec3 vel;
    glm::rvec3 angvel;
    glm::rvec3 acc;
    glm::rvec3 angacc;
    glm::rvec3 f_ext;
    glm::rvec3 tau_ext;
    glm::rvec3 f_c;
    glm::rvec3 tau_c;
    Id<Material> mat_id;
    btCollisionObject* bt_collision_object;
    bool is_static = false;

    void init(RigidBodySpec rb_spec);

    void init(Id<RigidBody> rb_id, RigidBodySpec rb_spec, Id<Material> mat_id,
              btCollisionWorld* bt_collision_world,
              int col_filter_group_mask = btBroadphaseProxy::DefaultFilter,
              int col_filter_mask = btBroadphaseProxy::AllFilter);

    void release(btCollisionWorld* bt_world);

    void reset();
    void randomize_positions();

    const RigidBodySpec& get_spec() const { return spec; }
    RigidBodySpec& get_spec_mut() { return spec; }

    void update_colliders();
    void forward_dynamics(const rvec3& gravity);
    void integrate(real dt);
    void integrate_positions(real dt);
    void integrate_velocities(real dt);
    void simulate(const rvec3& gravity, real dt);
};

}
#endif //ARTSIM_RIGID_BODY_H
