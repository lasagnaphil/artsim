//
// Created by lasagnaphil on 8/21/21.
//

#include <artsim/rigid_body.h>

#include <artsim/types.h>
#include <artsim/collision_shape.h>
#include <artsim/artsim.h>
#include <artsim/material.h>

#include <BulletCollision/CollisionDispatch/btCollisionWorld.h>

namespace artsim {

void RigidBody::init(RigidBodySpec rb_spec) {
    this->spec = rb_spec;
    pos = glm::rvec3(0);
    vel = glm::rvec3(0);
    rot = glm::identity<glm::rquat>();
    angvel = glm::rvec3(0);
}

void RigidBody::init(Id<RigidBody> rb_id, RigidBodySpec rb_spec, Id<Material> mat_id,
                     btCollisionWorld* bt_collision_world) {
    init(rb_spec);
    this->mat_id = mat_id;
    auto col_shape = spec.col_shape;
    if (col_shape.type != CollisionShape::Type::Mesh) {
        BodyLinkId body_id = BodyLinkId::from_rigid_body(rb_id);
        bt_collision_object = new btCollisionObject;
        bt_collision_object->setCollisionShape(spec.col_shape.bt_shape);
        bt_collision_object->setWorldTransform(btTransform::getIdentity());
        bt_collision_object->setUserIndex(body_id.index);
        bt_collision_object->setUserIndex2(body_id.generation);
        if (col_shape.type == CollisionShape::Type::Ground) {
            bt_collision_world->addCollisionObject(bt_collision_object, btBroadphaseProxy::DefaultFilter, btBroadphaseProxy::AllFilter);
        }
        else {
            bt_collision_world->addCollisionObject(bt_collision_object, 0b1000000, ~0);
        }
    }
}

void RigidBody::release(btCollisionWorld* bt_world) {
    bt_world->removeCollisionObject(bt_collision_object);
    delete bt_collision_object;
}

}