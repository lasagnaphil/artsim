//
// Created by lasagnaphil on 3/27/21.
//

#ifndef ARTSIM_PBD_H
#define ARTSIM_PBD_H

#include <memory>

#include <artsim/core/arena.h>
#include <artsim/math/se3.h>
#include <artsim/types.h>
#include <artsim/artsim.h>

#include <BulletCollision/CollisionDispatch/btCollisionWorld.h>
#include <BulletCollision/BroadphaseCollision/btDbvtBroadphase.h>
#include <BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.h>
#include <BulletCollision/CollisionShapes/btStaticPlaneShape.h>
#include <BulletCollision/CollisionShapes/btSphereShape.h>
#include <BulletCollision/CollisionShapes/btBoxShape.h>

namespace artsim {

DEFINE_TYPEID(btBoxShape, 1)

struct CollisionShapeId : public Id<void> {
    CollisionShapeId() = default;

    template <class T>
    CollisionShapeId(Id<T> id) {
        index = id.index;
        type = id.type;
        generation = id.generation;
    };

    template <class T>
    operator Id<T>() const {
        assert(type == TypeID<btBoxShape>()());
        return Id<T> {index, type, generation};
    }
};

struct PBDRigidBody {
    glmx::tsmat3x3<real> inertia, inv_inertia;
    real mass, inv_mass;
    Id<btCollisionObject> col_obj;
    Id<btCollisionShape> col_shape;

    bool is_dynamic = false;

    glm::tvec3<real> f_ext;
    glm::tvec3<real> tau_ext;

    glm::tvec3<real> pos;
    glm::tquat<real> rot;
    glm::tvec3<real> vel;
    glm::tvec3<real> angvel;

    glm::tvec3<real> prev_pos;
    glm::tquat<real> prev_rot;
};

enum class PBDConstraintType {
    RevoluteJoint, SphericalJoint
};

/*
struct PBDFixPositionConstraint {
    Id<PBDRigidBody> rb_id;
    glm::tvec3<real> offset;
    glm::tvec3<real> pos;
    glm::tvec3<real> axis;
    real limit_min, limit_max;

    real lambda;
    glm::tvec3<real> force;
};
 */

struct PBDFixRotationConstraint {
    Id<PBDRigidBody> rb_id;
    glm::tvec3<real> offset;
    glm::tquat<real> rot;

    real lambda;
    glm::tvec3<real> torque;
};

struct PBDRevoluteJointConstraint {
    Id<PBDRigidBody> rb_id1, rb_id2;
    glm::tvec3<real> offset1, offset2;
    glm::tvec3<real> axis;
    real limit_min, limit_max;

    real pos_lambda, rot_lambda;
};

struct PBDSphericalJointConstraint {
    Id<PBDRigidBody> rb_id1, rb_id2;
    glm::tvec3<real> offset1, offset2;
    glm::tvec3<real> twist_axis;
    real twist_limit_min, twist_limit_max;
    real swing_limit_min, swing_limit_max;

    real pos_lambda, swing_rot_lambda, twist_rot_lambda;
};

struct PBDConstraint {
    PBDConstraintType type;
    real compliance = 0.0f;
    union {
        // PBDFixPositionConstraint fix_position;
        // PBDFixRotationConstraint fix_rotation;
        PBDRevoluteJointConstraint revolute_joint;
        PBDSphericalJointConstraint spherical_joint;
    };
};

struct PBDCollisionShapes {
    Arena<btBoxShape> box_shapes;

    void clear() {
        box_shapes.clear();
    }

    CollisionShapeId make_box(glm::rvec3 half_extents) {
        btBoxShape shape(btconv(half_extents));
        return CollisionShapeId(box_shapes.insert(shape));
    }

    btCollisionShape* get(CollisionShapeId id) {
        if (id.type == TypeID<btBoxShape>()()) {
            return box_shapes.get(id);
        }
        else {
            return nullptr;
        }
    }

    void erase(CollisionShapeId id) {
        if (id.type == TypeID<btBoxShape>()()) {
            box_shapes.release(id);
        }
    }
};

class PBDWorld {
private:
    Arena<PBDRigidBody> rigid_bodies;
    Arena<PBDConstraint> constraints;
    glm::rvec3 gravity = {0.0, -9.8, 0.0};

    PBDCollisionShapes bt_collision_shapes;
    Arena<btCollisionObject> bt_collision_objects;
    std::unique_ptr<btCollisionConfiguration> bt_collision_config;
    std::unique_ptr<btDispatcher> bt_dispatcher;
    std::unique_ptr<btBroadphaseInterface> bt_broadphase;
    std::unique_ptr<btCollisionWorld> bt_world;

public:
    PBDWorld();

    void reset();

    Id<PBDRigidBody> make_cube(glm::rvec3 size, real mass,
                               glm::rvec3 pos = {}, glm::rquat rot = glm::identity<glm::rquat>(),
                               glm::rvec3 vel = {}, glm::rvec3 angvel = {},
                               glm::rvec3 f_ext = {}, glm::rvec3 tau_ext = {});

    // Id<PBDConstraint> make_positional_constraint(Id<PBDRigidBody> rb_id, glm::rvec3 offset, glm::rvec3 pos);

    Id<PBDConstraint> make_revolute_joint_constraint(Id<PBDRigidBody> rb_id1, Id<PBDRigidBody> rb_id2,
                                                     real compliance,
                                                     glm::rvec3 offset1, glm::rvec3 offset2,
                                                     glm::rvec3 axis,
                                                     real limit_min = -REAL_MAX, real limit_max = REAL_MAX);

    Id<PBDConstraint> make_spherical_joint_constraint(Id<PBDRigidBody> rb_id1, Id<PBDRigidBody> rb_id2,
                                                      real compliance,
                                                      glm::rvec3 offset1, glm::rvec3 offset2,
                                                      glm::rvec3 twist_axis,
                                                      real twist_limit_min = -REAL_MAX, real twist_limit_max = REAL_MAX,
                                                      real swing_limit_min = -REAL_MAX, real swing_limit_max = REAL_MAX);

    PBDRigidBody* get_rigid_body(Id<PBDRigidBody> id) {
        return rigid_bodies.get(id);
    }
    const PBDRigidBody* get_rigid_body(Id<PBDRigidBody> id) const {
        return rigid_bodies.get(id);
    }
    PBDConstraint* get_constraint(Id<PBDConstraint> id) {
        return constraints.get(id);
    }
    const PBDConstraint* get_constraint(Id<PBDConstraint> id) const {
        return constraints.get(id);
    }

    void remove_rigid_body(Id<PBDRigidBody> id) {
        auto* rb = rigid_bodies.get(id);
        auto* col_obj = bt_collision_objects.get(rb->col_obj);
        bt_world->removeCollisionObject(col_obj);
        bt_collision_shapes.erase(rb->col_shape);
        rigid_bodies.erase(id);
    }

    void remove_constraint(Id<PBDConstraint> id) {
        constraints.erase(id);
    }

    int get_num_rigid_bodies() { return rigid_bodies.size(); }
    PBDRigidBody* get_rigid_body_buf() { return rigid_bodies.get_items_buf(); }
    const PBDRigidBody* get_rigid_body_buf() const { return rigid_bodies.get_items_buf(); }

    void simulate(real dt, int num_substeps = 20);

private:
    void reset_lambdas();
    void collect_collision_pairs();
    void solve_positions(real h);
    void solve_velocities(real h);
};

}

#endif //ARTSIM_PBD_H
