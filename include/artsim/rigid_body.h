//
// Created by lasagnaphil on 8/21/21.
//

#ifndef ARTSIM_RIGID_BODY_H
#define ARTSIM_RIGID_BODY_H

#include <artsim/math/se3.h>
#include <artsim/types.h>
#include <artsim/collision_shape.h>
#include <artsim/material.h>

class btCollisionShape;
class btCollisionWorld;
class btCollisionObject;

namespace artsim {

struct RigidBodySpec {
    glmx::tsmat3x3<real> inertia;
    real mass;
    CollisionShape col_shape;
    glmx::ttransform<real> global_trans;
    bool is_static = false;
};

struct RigidBody {
    RigidBodySpec spec;
    glm::rvec3 pos;
    glm::rvec3 vel;
    glm::rquat rot;
    glm::rvec3 angvel;
    Id<Material> mat_id;
    btCollisionObject* bt_collision_object;

    void init(RigidBodySpec rb_spec);

    void init(Id<RigidBody> rb_id, RigidBodySpec rb_spec, Id<Material> mat_id,
              btCollisionWorld* bt_collision_world);

    void release(btCollisionWorld* bt_world);

    const RigidBodySpec& get_spec() const { return spec; }
    RigidBodySpec& get_spec_mut() { return spec; }
};

}
#endif //ARTSIM_RIGID_BODY_H
