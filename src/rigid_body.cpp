//
// Created by lasagnaphil on 8/21/21.
//

#include <artsim/rigid_body.h>

#include <artsim/types.h>
#include <artsim/math/bullet.h>
#include <artsim/math/eigen.h>
#include <artsim/math/se3.h>
#include <artsim/collision/collision_shape.h>
#include <artsim/artsim.h>
#include <artsim/material.h>

#include <BulletCollision/CollisionDispatch/btCollisionWorld.h>

#include <random>
#include <ctime>

namespace artsim {

void RigidBody::reset() {
    world_trans = glmx::rquat_transform(glmx::IDENTITY);
    body_vel = glmx::rscrew(glmx::IDENTITY);
    body_acc = glmx::rscrew(glmx::IDENTITY);
    body_f_ext = glmx::rscrew(glmx::IDENTITY);
    body_f_c = glmx::rscrew(glmx::IDENTITY);
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
        world_trans.q = glmx::exp(len * normalize(dir));
    }
    {
        world_trans.v[0] = std::uniform_real_distribution<real>(-0.2*pi, 0.2*pi)(engine);
        world_trans.v[1] = std::uniform_real_distribution<real>(2-0.2*pi, 2+0.2*pi)(engine);
        world_trans.v[2] = std::uniform_real_distribution<real>(-0.2*pi, 0.2*pi)(engine);
    }
}

void RigidBody::update_colliders() {
    auto shape = world->get_shape(shape_id);
    switch (shape->type) {
        case CollisionShape::Type::Ground: {
            constexpr auto inf = std::numeric_limits<real>::infinity();
            bounds = glmx::tbox<3, real>(glm::rvec3(-inf, 0, -inf), glm::rvec3(inf, 0, inf));
        } break;
        case CollisionShape::Type::Sphere: {
            bounds = glmx::tbox<3, real>(-shape->scale, shape->scale);
        } break;
        case CollisionShape::Type::Mesh: {
            auto mesh = world->get_collision_mesh(shape->mesh);
            bounds = glmx::tbox<3, real>();
            auto& vertices = mesh->mesh->vertex_data();
            for (auto& v : vertices) {
                bounds.extend(to_glm_vec(v));
            }
        } break;
    }
    bounds_center = bounds.center();
}

void RigidBody::forward_dynamics(const rvec3& gravity) {
    auto tau = adT(body_vel, I * body_vel) + body_f_ext + body_f_c;
    body_acc = inverse(I) * tau;
}

void RigidBody::integrate(real dt) {
    integrate_velocities(dt);
    integrate_positions(dt);
}

void RigidBody::integrate_positions(real dt) {
    auto world_vel = world_trans.q * body_vel.v;
    auto world_angvel = body_vel.w;
    world_trans.v += world_vel * dt;
    world_trans.q += glm::rquat(0, real(0.5)*dt*world_angvel) * world_trans.q;
    world_trans.q = glm::normalize(world_trans.q);
}

void RigidBody::integrate_velocities(real dt) {
    body_vel += body_acc * dt;
}

void RigidBody::simulate(const rvec3& gravity, real dt) {
    forward_dynamics(gravity);
    integrate(dt);
}

}