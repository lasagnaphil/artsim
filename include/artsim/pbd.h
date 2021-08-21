//
// Created by lasagnaphil on 3/27/21.
//

#ifndef ARTSIM_PBD_H
#define ARTSIM_PBD_H

#include <memory>

#include <artsim/core/arena.h>
#include <artsim/math/se3.h>
#include <artsim/math/bullet.h>
#include <artsim/types.h>
#include <artsim/artsim.h>

#include <LinearMath/btPoolAllocator.h>
#include <BulletCollision/CollisionDispatch/btCollisionWorld.h>
#include <BulletCollision/BroadphaseCollision/btDbvtBroadphase.h>
#include <BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.h>
#include <BulletCollision/CollisionShapes/btStaticPlaneShape.h>
#include <BulletCollision/CollisionShapes/btSphereShape.h>
#include <BulletCollision/CollisionShapes/btBoxShape.h>

namespace artsim {

struct PBDMaterial {
    real mu_static;
    real mu_dynamic;
    real restitution;
};

struct PBDRigidBody {
    glmx::tsmat3x3<real> inertia, inv_inertia;
    real mass, inv_mass;
    Id<PBDMaterial> mat_id;
    btCollisionShape* col_shape;
    btCollisionObject* col_obj;

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

struct PBDRevoluteJointConstraint {
    Id<PBDRigidBody> rb_id1, rb_id2;
    glm::tvec3<real> offset1, offset2;
    glm::tvec3<real> axis;
    real limit_min, limit_max;
    real damping;

    real pos_lambda, rot_lambda, rot_limit_lambda;
};

struct PBDSphericalJointConstraint {
    Id<PBDRigidBody> rb_id1, rb_id2;
    glm::tvec3<real> offset1, offset2;
    glm::tvec3<real> twist_axis;
    real twist_limit_min, twist_limit_max;
    real swing_limit_min, swing_limit_max;
    real damping;

    real pos_lambda, swing_rot_lambda, twist_rot_lambda;
};

struct PBDRigidRigidContactConstraint {
    Id<PBDRigidBody> rb_id1, rb_id2;
    glm::tvec3<real> p1, p2;
    glm::tvec3<real> r1, r2;
    glm::tvec3<real> normal;

    real normal_lambda, tangent_lambda;
};

struct PBDConstraint {
    PBDConstraintType type;
    real compliance = 0.0f;
    union {
        PBDRevoluteJointConstraint revolute_joint;
        PBDSphericalJointConstraint spherical_joint;
        PBDRigidRigidContactConstraint contact;
    };
};

#if 0
class PBDCollisionShapes {
private:
    btStaticPlaneShape* plane_shape = nullptr;
    btPoolAllocator box_shape_allocator;

public:
    PBDCollisionShapes(int max_box_shapes = 65536)
        : box_shape_allocator(sizeof(btBoxShape), max_box_shapes)
    {}

    void clear() {
        delete plane_shape;
        plane_shape = nullptr;
        box_shape_allocator = btPoolAllocator(sizeof(btBoxShape), box_shape_allocator.getMaxCount());
    }

    btStaticPlaneShape* make_static_plane(glm::rvec3 normal, real constant) {
        if (plane_shape) {
            return nullptr;
        }
        else {
            plane_shape = new btStaticPlaneShape(btconv(normal), constant);
            return plane_shape;
        }
    }

    btBoxShape* make_box(glm::rvec3 half_extents) {
        auto ptr = reinterpret_cast<btBoxShape*>(box_shape_allocator.allocate(1));
        new (ptr) btBoxShape(btconv(half_extents));
        return ptr;
    }

    void erase(btStaticPlaneShape* shape) {
        delete shape;
        plane_shape = nullptr;
    }
    void erase(btBoxShape* shape) {
        box_shape_allocator.freeMemory(shape);
    }

    void erase(btCollisionShape* shape) {
        switch (shape->getShapeType()) {
            case STATIC_PLANE_PROXYTYPE: {
                erase(dynamic_cast<btStaticPlaneShape*>(shape));
            }
            case BOX_SHAPE_PROXYTYPE: {
                erase(dynamic_cast<btCollisionShape*>(shape));
            } break;
        }
    }
};


class PBDCollisionObjects {
private:
    btPoolAllocator allocator;

public:
    PBDCollisionObjects(int max_col_objects = 65536) : allocator(sizeof(btCollisionObject), max_col_objects) {}

    void clear() {
        allocator = btPoolAllocator(sizeof(btCollisionObject), allocator.getMaxCount());
    }

    btCollisionObject* make(glmx::rquat_transform trans, btCollisionShape* shape) {
        auto obj = reinterpret_cast<btCollisionObject*>(allocator.allocate(1));
        obj->setCollisionShape(shape);
        obj->setWorldTransform(btconv(trans));
        return obj;
    }

    void erase(btCollisionObject* obj) {
        allocator.freeMemory(obj);
    }
};
#else
class PBDCollisionShapes {
public:
    void clear() {
    }

    btStaticPlaneShape* make_static_plane(glm::rvec3 normal, real constant) {
        auto plane_shape = new btStaticPlaneShape(btconv(normal), constant);
        return plane_shape;
    }

    btBoxShape* make_box(glm::rvec3 half_extents) {
        auto box_shape = new btBoxShape(btconv(half_extents));
        return box_shape;
    }

    void erase(btStaticPlaneShape* shape) {
        delete shape;
    }
    void erase(btBoxShape* shape) {
        delete shape;
    }
    void erase(btCollisionShape* shape) {
        delete shape;
    }
};

class PBDCollisionObjects {
public:
    void clear() {

    }
    btCollisionObject* make() {
        auto obj = new btCollisionObject();
        return obj;
    }

    void erase(btCollisionObject* obj) {
        delete obj;
    }
};
#endif

class PBDWorld {
private:
    Arena<PBDRigidBody> rigid_bodies;
    Arena<PBDMaterial> materials;
    Arena<PBDConstraint> constraints;
    glm::rvec3 gravity = {0.0, -9.8, 0.0};

    std::vector<PBDRigidRigidContactConstraint> rb_rb_contact_constraints;

    PBDCollisionShapes bt_collision_shapes;
    PBDCollisionObjects bt_collision_objects;

    std::unique_ptr<btCollisionConfiguration> bt_collision_config;
    std::unique_ptr<btDispatcher> bt_dispatcher;
    std::unique_ptr<btBroadphaseInterface> bt_broadphase;
    std::unique_ptr<btCollisionWorld> bt_world;

public:
    PBDWorld();

    void reset();

    void set_debug_drawer(btIDebugDraw* debug_draw_interface) { bt_world->setDebugDrawer(debug_draw_interface); }
    void debug_draw() { bt_world->debugDrawWorld(); }

    Id<PBDRigidBody> make_cube(glm::rvec3 size, real mass, Id<PBDMaterial> mat_id,
                               int col_filter_group = btBroadphaseProxy::DefaultFilter,
                               int col_filter_mask = btBroadphaseProxy::AllFilter,
                               glm::rvec3 pos = {}, glm::rquat rot = glm::identity<glm::rquat>(),
                               glm::rvec3 vel = {}, glm::rvec3 angvel = {},
                               glm::rvec3 f_ext = {}, glm::rvec3 tau_ext = {});

    Id<PBDRigidBody> make_static_plane(Id<PBDMaterial> mat_id, glm::rvec3 normal, real constant);

    Id<PBDMaterial> make_material(real mu_static, real mu_dynamic, real restitution);

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
        bt_world->removeCollisionObject(rb->col_obj);
        bt_collision_shapes.erase(rb->col_shape);
        rigid_bodies.erase(id);
    }

    void remove_constraint(Id<PBDConstraint> id) {
        constraints.erase(id);
    }

    int get_num_rigid_bodies() { return rigid_bodies.size(); }
    PBDRigidBody* get_rigid_body_buf() { return rigid_bodies.get_items_buf(); }
    const PBDRigidBody* get_rigid_body_buf() const { return rigid_bodies.get_items_buf(); }

    int get_num_rb_rb_collision_constraints() { return rb_rb_contact_constraints.size(); }
    PBDRigidRigidContactConstraint* get_rb_rb_collision_constraint_buf() { return rb_rb_contact_constraints.data(); }

    void simulate(real dt, int num_substeps = 20);

private:
    void reset_lambdas();
    void collect_collision_pairs();
    void solve_positions(real h);
    void solve_velocities(real h);
};

}

#endif //ARTSIM_PBD_H
