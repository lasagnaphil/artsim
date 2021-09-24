//
// Created by lasagnaphil on 8/21/21.
//

#include <artsim/rigid_body.h>

#include <artsim/types.h>
#include <artsim/math/bullet.h>
#include <artsim/collision/collision_shape.h>
#include <artsim/artsim.h>
#include <artsim/material.h>

#include <BulletCollision/CollisionDispatch/btCollisionWorld.h>

#include <random>
#include <ctime>

namespace artsim {

void RigidBody::reset() {
    pos = {};
    rot = glm::identity<glm::rquat>();
    vel = {};
    angvel = {};
    acc = {};
    angacc = {};
    f_ext = {};
    tau_ext = {};
    f_c = {};
    tau_c = {};
}

void RigidBody::randomize_positions() {
    thread_local std::default_random_engine engine(std::time(nullptr));

    const real pi = glm::pi<real>();
    {
        real len = std::uniform_real_distribution<real>(-0.2*pi, 0.2*pi)(engine);
        glm::tvec3<real> dir = glm::tvec3<real>(
                std::uniform_real_distribution<real>(-1, 1)(engine),
                std::uniform_real_distribution<real>(-1, 1)(engine),
                std::uniform_real_distribution<real>(-1, 1)(engine)
                );
        rot = glmx::exp(len * normalize(dir));
    }
    {
        pos[0] = std::uniform_real_distribution<real>(-0.2*pi, 0.2*pi)(engine);
        pos[1] = std::uniform_real_distribution<real>(2-0.2*pi, 2+0.2*pi)(engine);
        pos[2] = std::uniform_real_distribution<real>(-0.2*pi, 0.2*pi)(engine);
    }
}

void RigidBody::update_colliders() {
    bt_collision_object->setWorldTransform(btconv(rquat_transform(pos, rot)));
}

void RigidBody::forward_dynamics(const rvec3& gravity) {
    acc = gravity + (f_ext + f_c) / mass;
    angacc = inv_inertia * (tau_ext + tau_c - glm::cross(angvel, (inertia * angvel)));
}

void RigidBody::integrate(real dt) {
    integrate_velocities(dt);
    integrate_positions(dt);
}

void RigidBody::integrate_positions(real dt) {
    pos += vel * dt;
    rot += glm::rquat(0, real(0.5)*dt*angvel) * rot;
    rot = glm::normalize(rot);
}

void RigidBody::integrate_velocities(real dt) {
    vel += acc * dt;
    angvel += angacc * dt;
}

void RigidBody::simulate(const rvec3& gravity, real dt) {
    forward_dynamics(gravity);
    integrate(dt);
}

}