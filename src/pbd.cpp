//
// Created by lasagnaphil on 3/27/21.
//

#include <artsim/pbd.h>
#include <glm/gtx/string_cast.hpp>
#include <BulletCollision/BroadphaseCollision/btAxisSweep3.h>

namespace artsim {

PBDWorld::PBDWorld() {
    bt_collision_config = std::make_unique<btDefaultCollisionConfiguration>();
    bt_dispatcher = std::make_unique<btCollisionDispatcher>(bt_collision_config.get());
    bt_broadphase = std::make_unique<btAxisSweep3>(btVector3(-50, -50, -50), btVector3(50, 50, 50));
    bt_world = std::make_unique<btCollisionWorld>(bt_dispatcher.get(), bt_broadphase.get(), bt_collision_config.get());
}

void PBDWorld::reset() {
    for (auto& rb : rigid_bodies) {
        bt_world->removeCollisionObject(rb.col_obj);
    }
    bt_collision_objects.clear();
    bt_collision_shapes.clear();
    rigid_bodies.clear();
    constraints.clear();
}

Id<PBDRigidBody> PBDWorld::make_cube(glm::rvec3 size, artsim::real mass, Id<PBDMaterial> mat_id,
                                     int col_filter_group, int col_filter_mask,
                                     glm::rvec3 pos, glm::rquat rot,
                                     glm::rvec3 vel, glm::rvec3 angvel,
                                     glm::rvec3 f_ext, glm::rvec3 tau_ext) {
    PBDRigidBody rb;
    const glm::tvec3<real>& s = size;
    glm::tvec3<real> I = mass * glm::tvec3<real>(s.y*s.y + s.z*s.z, s.z*s.z + s.x*s.x, s.x*s.x + s.y*s.y) / real(12);
    rb.inertia = glmx::tsmat3x3<real>(I.x, I.y, I.z, 0, 0, 0);
    rb.inv_inertia = glmx::inverse(rb.inertia);
    rb.mass = mass;
    rb.inv_mass = 1.0 / mass;
    rb.mat_id = mat_id;
    rb.col_shape = bt_collision_shapes.make_box(real(0.5) * s);
    rb.col_obj = bt_collision_objects.make();
    rb.is_dynamic = true;
    rb.pos = pos;
    rb.rot = rot;
    rb.vel = vel;
    rb.angvel = angvel;
    rb.f_ext = f_ext;
    rb.tau_ext = tau_ext;
    rb.prev_pos = pos;
    rb.prev_rot = rot;
    Id<PBDRigidBody> id = rigid_bodies.insert(rb);

    auto [user_id1, user_id2] = id.to_int32s();
    rb.col_obj->setCollisionShape(rb.col_shape);
    rb.col_obj->setWorldTransform(btconv(glmx::rquat_transform(rb.pos, rb.rot)));
    rb.col_obj->setUserIndex(user_id1);
    rb.col_obj->setUserIndex2(user_id2);
    bt_world->addCollisionObject(rb.col_obj, col_filter_group, col_filter_mask);
    auto broadphase_handle = rb.col_obj->getBroadphaseHandle();
    if (broadphase_handle) {
        btVector3 aabb_min, aabb_max;
        bt_broadphase->setAabb(broadphase_handle, aabb_min, aabb_max, bt_dispatcher.get());
    }


    return id;
}

Id<PBDRigidBody> PBDWorld::make_static_plane(Id<PBDMaterial> mat_id, glm::rvec3 normal, real constant) {
    PBDRigidBody rb;
    rb.inertia = REAL_MAX;
    rb.inv_inertia = 0;
    rb.mass = REAL_MAX;
    rb.inv_mass = 0;
    rb.mat_id = mat_id;
    rb.col_shape = bt_collision_shapes.make_static_plane(normal, constant);
    rb.col_obj = bt_collision_objects.make();
    rb.is_dynamic = false;
    rb.pos = {};
    rb.rot = glm::identity<glm::quat>();
    rb.vel = {};
    rb.angvel = {};
    rb.f_ext = {};
    rb.tau_ext = {};
    rb.prev_pos = {};
    rb.prev_rot = {};
    Id<PBDRigidBody> id = rigid_bodies.insert(rb);

    auto [user_id1, user_id2] = id.to_int32s();
    rb.col_obj->setCollisionShape(rb.col_shape);
    rb.col_obj->setUserIndex(user_id1);
    rb.col_obj->setUserIndex2(user_id2);
    bt_world->addCollisionObject(rb.col_obj);

    return id;
}

Id<PBDMaterial> PBDWorld::make_material(real mu_static, real mu_dynamic, real restitution) {
    PBDMaterial mat;
    mat.mu_static = mu_static;
    mat.mu_dynamic = mu_dynamic;
    mat.restitution = restitution;
    return materials.insert(mat);
}

Id<PBDConstraint> PBDWorld::make_revolute_joint_constraint(Id<PBDRigidBody> rb_id1, Id<PBDRigidBody> rb_id2,
                                                           real compliance,
                                                           glm::rvec3 offset1, glm::rvec3 offset2,
                                                           glm::rvec3 axis, real limit_min, real limit_max) {
    if (!rigid_bodies.is_valid(rb_id1) || !rigid_bodies.is_valid(rb_id2)) return {};
    PBDConstraint cons;
    cons.type = PBDConstraintType::RevoluteJoint;
    cons.compliance = compliance;
    cons.revolute_joint.rb_id1 = rb_id1;
    cons.revolute_joint.rb_id2 = rb_id2;
    cons.revolute_joint.offset1 = offset1;
    cons.revolute_joint.offset2 = offset2;
    cons.revolute_joint.axis = axis;
    cons.revolute_joint.limit_min = limit_min;
    cons.revolute_joint.limit_max = limit_max;
    cons.revolute_joint.damping = 0;
    cons.revolute_joint.pos_lambda = 0;
    cons.revolute_joint.rot_lambda = 0;
    cons.revolute_joint.rot_limit_lambda = 0;
    return constraints.insert(cons);
}

Id<PBDConstraint> PBDWorld::make_spherical_joint_constraint(Id<PBDRigidBody> rb_id1, Id<PBDRigidBody> rb_id2,
                                                            real compliance,
                                                            glm::rvec3 offset1, glm::rvec3 offset2,
                                                            glm::rvec3 twist_axis, real twist_limit_min, real twist_limit_max,
                                                            real swing_limit_min, real swing_limit_max) {
    if (!rigid_bodies.is_valid(rb_id1) || !rigid_bodies.is_valid(rb_id2)) return {};
    PBDConstraint cons;
    cons.type = PBDConstraintType::SphericalJoint;
    cons.compliance = compliance;
    cons.spherical_joint.rb_id1 = rb_id1;
    cons.spherical_joint.rb_id2 = rb_id2;
    cons.spherical_joint.offset1 = offset1;
    cons.spherical_joint.offset2 = offset2;
    cons.spherical_joint.twist_axis = twist_axis;
    cons.spherical_joint.twist_limit_min = twist_limit_min;
    cons.spherical_joint.twist_limit_max = twist_limit_max;
    cons.spherical_joint.swing_limit_min = swing_limit_min;
    cons.spherical_joint.swing_limit_max = swing_limit_max;
    cons.spherical_joint.damping = 0;
    cons.spherical_joint.pos_lambda = 0;
    cons.spherical_joint.swing_rot_lambda = 0;
    cons.spherical_joint.twist_rot_lambda = 0;
    return constraints.insert(cons);
}

void PBDWorld::simulate(real dt, int num_substeps) {
    PBDRigidBody* rbs = rigid_bodies.get_items_buf();
    int num_rbs = rigid_bodies.size();

    reset_lambdas();
    collect_collision_pairs();

    real h = dt / num_substeps;
    for (int iter = 0; iter < num_substeps; iter++) {
        for (int rb_idx = 0; rb_idx < num_rbs; rb_idx++) {
            PBDRigidBody& rb = rbs[rb_idx];
            if (rb.is_dynamic) {
                rb.prev_pos = rb.pos;
                rb.vel += h * (gravity + rb.inv_mass * rb.f_ext);
                rb.pos += h * rb.vel;

                rb.prev_rot = rb.rot;
                rb.angvel += h*(rb.inv_inertia*(rb.tau_ext - glm::cross(rb.angvel, rb.inertia * rb.angvel)));
                rb.rot += glm::rquat(0, real(0.5)*h*rb.angvel) * rb.rot;
                rb.rot = glm::normalize(rb.rot);
            }
        }
        solve_positions(h);
        for (int rb_idx = 0; rb_idx < num_rbs; rb_idx++) {
            PBDRigidBody& rb = rbs[rb_idx];
            if (rb.is_dynamic) {
                rb.vel = (rb.pos - rb.prev_pos) / h;
                glm::rquat dq = rb.rot * glm::inverse(rb.prev_rot);
                rb.angvel = (real(2.0)/h) * glm::rvec3(dq.x, dq.y, dq.z);
                rb.angvel = dq.w >= 0? rb.angvel : -rb.angvel;
            }
        }
        solve_velocities(h);
    }
}

void PBDWorld::reset_lambdas() {
    for (auto& con : constraints) {
        switch (con.type) {
            case PBDConstraintType::RevoluteJoint: {
                con.revolute_joint.pos_lambda = 0;
                con.revolute_joint.rot_lambda = 0;
            } break;
            case PBDConstraintType::SphericalJoint: {
                con.spherical_joint.pos_lambda = 0;
                con.spherical_joint.swing_rot_lambda = 0;
                con.spherical_joint.twist_rot_lambda = 0;
            }
        }
    }
}

void PBDWorld::collect_collision_pairs() {
    rb_rb_contact_constraints.clear();
    for (auto& rb : rigid_bodies) {
        if (rb.is_dynamic) {
            rb.col_obj->setWorldTransform(btconv(glmx::rquat_transform(rb.pos, rb.rot)));
        }
    }
    bt_world->performDiscreteCollisionDetection();

    auto dispatcher = bt_world->getDispatcher();
    int num_manifolds = dispatcher->getNumManifolds();

    for (int i = 0; i < num_manifolds; i++) {
        btPersistentManifold* manifold = dispatcher->getManifoldByIndexInternal(i);
        int num_contacts = manifold->getNumContacts();
        if (num_contacts == 0) continue;

        const btCollisionObject* body1 = manifold->getBody0();
        const btCollisionObject* body2 = manifold->getBody1();
        Id<PBDRigidBody> rb_id1 = Id<PBDRigidBody>::from_int32s(body1->getUserIndex(), body1->getUserIndex2());
        Id<PBDRigidBody> rb_id2 = Id<PBDRigidBody>::from_int32s(body2->getUserIndex(), body2->getUserIndex2());
        for (int j = 0; j < num_contacts; j++) {
            auto& pt = manifold->getContactPoint(j);
            if (pt.getDistance() < 0.f) {
                PBDRigidRigidContactConstraint con;
                con.rb_id1 = rb_id1;
                con.rb_id2 = rb_id2;
                con.p1 = glmconv(pt.m_positionWorldOnA);
                con.p2 = glmconv(pt.m_positionWorldOnB);
                con.r1 = glmconv(pt.m_localPointA);
                con.r2 = glmconv(pt.m_localPointB);
                con.normal = glmconv(pt.m_normalWorldOnB);
                con.normal_lambda = 0;
                con.tangent_lambda = 0;
                rb_rb_contact_constraints.push_back(con);
                auto p1_txt = glm::to_string(con.p1);
                auto p2_txt = glm::to_string(con.p2);
                printf("Contact at %s, %s with depth=%f\n", p1_txt.c_str(), p2_txt.c_str(), pt.getDistance());
            }
        }
    }
}

void project_positions(PBDRigidBody& rb1, PBDRigidBody& rb2, glm::rvec3 dx,
                       real alpha, glm::rvec3 r1, glm::rvec3 r2, real& lambda) {
    if (!rb1.is_dynamic && !rb2.is_dynamic) return;

    real c = glm::length(dx);
    if (c <= glm::epsilon<real>()) return;
    glm::rvec3 n = dx / c;
    real w1 = rb1.inv_mass + glmx::quadratic_form(rb1.inv_inertia, glm::cross(r1, n));
    real w2 = rb2.inv_mass + glmx::quadratic_form(rb2.inv_inertia, glm::cross(r2, n));
    real w_tot = int(rb1.is_dynamic) * w1 + int(rb2.is_dynamic) * w2;
    real dlambda = (-c - alpha * lambda) / (w_tot + alpha);
    lambda += dlambda;

    glm::rvec3 p = dlambda * n;

    if (rb1.is_dynamic) {
        rb1.pos += rb1.inv_mass * p;
        rb1.rot += real(0.5) * (glm::rquat(0, rb1.inv_inertia * glm::cross(r1, p)) * rb1.rot);
        rb1.rot = glm::normalize(rb1.rot);
    }
    if (rb2.is_dynamic) {
        rb2.pos -= rb2.inv_mass * p;
        rb2.rot -= real(0.5) * (glm::rquat(0, rb2.inv_inertia * glm::cross(r2, p)) * rb2.rot);
        rb2.rot = glm::normalize(rb2.rot);
    }
}

void project_rotations(PBDRigidBody& rb1, PBDRigidBody& rb2, glm::rvec3 dq,
                       real alpha, real& lambda) {
    if (!rb1.is_dynamic && !rb2.is_dynamic) return;

    real theta = glm::length(dq);
    glm::rvec3 n = dq / theta;
    glm::rvec3 n_rel = glm::inverse(rb1.rot) * n;
    real w1 = glmx::quadratic_form(rb1.inv_inertia, n_rel);
    real w2 = glmx::quadratic_form(rb2.inv_inertia, n_rel);
    real w_tot = int(rb1.is_dynamic) * w1 + int(rb2.is_dynamic) * w2;
    real dlambda = (-theta - alpha * lambda) / (w_tot + alpha);
    lambda += dlambda;

    glm::rvec3 p = dlambda * n_rel;
    if (rb1.is_dynamic) {
        rb1.rot += real(0.5) * (glm::rquat(0, rb1.inv_inertia * p) * rb1.rot);
        rb1.rot = glm::normalize(rb1.rot);
    }
    if (rb2.is_dynamic) {
        rb2.rot -= real(0.5) * (glm::rquat(0, rb2.inv_inertia * p) * rb2.rot);
        rb2.rot = glm::normalize(rb2.rot);
    }
}

void project_rotations(PBDRigidBody& rb1, PBDRigidBody& rb2, real alpha, real& lambda) {
    glm::rvec3 dq = glmx::log(rb2.rot * glm::inverse(rb1.rot));
    project_rotations(rb1, rb2, dq, alpha, lambda);
}

bool limit_angle(glm::rvec3 n, glm::rvec3& n1, glm::rvec3 n2, real phi_min, real phi_max, glm::rvec3& dq) {
    constexpr real PI = glm::pi<real>();
    real phi = glm::asin(glm::dot(glm::cross(n1, n2), n));
    if (glm::dot(n1, n2) < 0) phi = 2*PI - phi;
    if (phi > PI) phi -= 2*PI;
    if (phi < -PI) phi += 2*PI;
    if (phi < phi_min || phi > phi_max) {
        phi = glm::clamp(phi, phi_min, phi_max);
        n1 = glm::angleAxis(phi, n) * n1;
        dq = glm::cross(n1, n2);
        return true;
    }
    else {
        return false;
    }
}

void PBDWorld::solve_positions(real h) {
    real h_sq = h*h;
    // Handle joint constraints
    for (auto& con : constraints) {
        real alpha = con.compliance / h_sq;
        switch (con.type) {
            case PBDConstraintType::RevoluteJoint: {
                auto& rev_con = con.revolute_joint;
                auto& rb1 = *rigid_bodies.get(rev_con.rb_id1);
                auto& rb2 = *rigid_bodies.get(rev_con.rb_id2);
                glm::rmat3 basis1 = glm::mat3_cast(rb1.rot);
                glm::rmat3 basis2 = glm::mat3_cast(rb2.rot);
                glm::rvec3 r1 = basis1*rev_con.offset1;
                glm::rvec3 r2 = basis2*rev_con.offset2;
                glm::rvec3 dx = (rb1.pos + r1) - (rb2.pos + r2);
                project_positions(rb1, rb2, dx, alpha, r1, r2, rev_con.pos_lambda);
                glm::rvec3 dq = glm::cross(basis1[0], basis2[0]);
                project_rotations(rb1, rb2, dq, alpha, rev_con.rot_lambda);
                if (limit_angle(basis1[0], basis1[1], basis2[1], rev_con.limit_min, rev_con.limit_max, dq)) {
                    project_rotations(rb1, rb2, dq, alpha, rev_con.rot_limit_lambda);
                }
            } break;
            case PBDConstraintType::SphericalJoint: {
                auto& sph_con = con.spherical_joint;
                auto& rb1 = *rigid_bodies.get(sph_con.rb_id1);
                auto& rb2 = *rigid_bodies.get(sph_con.rb_id2);
                glm::rmat3 basis1 = glm::mat3_cast(rb1.rot);
                glm::rmat3 basis2 = glm::mat3_cast(rb2.rot);
                glm::rvec3 dq_swing, dq_twist;
                glm::rvec3 r1 = basis1*sph_con.offset1;
                glm::rvec3 r2 = basis2*sph_con.offset2;
                glm::rvec3 dx = (rb1.pos + r1) - (rb2.pos + r2);
                project_positions(rb1, rb2, dx, alpha, r1, r2, sph_con.pos_lambda);
                if (limit_angle(glm::normalize(glm::cross(basis1[0], basis2[0])), basis1[1], basis2[1],
                                sph_con.swing_limit_min, sph_con.swing_limit_max, dq_swing)) {
                    project_rotations(rb1, rb2, dq_swing, alpha, sph_con.swing_rot_lambda);
                }
                glm::rvec3 n = glm::normalize(basis1[0] + basis2[0]);
                glm::rvec3 n1 = glm::normalize(basis1[1] - glm::dot(n, basis1[1])*n);
                glm::rvec3 n2 = glm::normalize(basis2[1] - glm::dot(n, basis2[1])*n);
                if (limit_angle(n, n1, n2, sph_con.twist_limit_min, sph_con.twist_limit_max, dq_twist)) {
                    project_rotations(rb1, rb2, dq_swing, alpha, sph_con.twist_rot_lambda);
                }
            } break;
        }
    }

    for (auto& con : rb_rb_contact_constraints) {
        auto& rb1 = *rigid_bodies.get(con.rb_id1);
        auto& rb2 = *rigid_bodies.get(con.rb_id2);
        auto& mat1 = *materials.get(rb1.mat_id);
        auto& mat2 = *materials.get(rb2.mat_id);
        real d = glm::dot(con.p1 - con.p2, con.normal);
        // real margin = rb1.col_shape->getMargin() + rb2.col_shape->getMargin();
        if (d <= 0) { continue; }
        glm::rvec3 dx = d * con.normal;
        project_positions(rb1, rb2, dx, 0, con.r1, con.r2, con.normal_lambda);

        glm::rvec3 p1_bar = rb1.prev_pos + rb1.prev_rot * con.r1;
        glm::rvec3 p2_bar = rb2.prev_pos + rb2.prev_rot * con.r2;
        glm::rvec3 dp = (con.p1 - p1_bar) - (con.p2 - p2_bar);
        glm::rvec3 dp_t = dp - glm::dot(dp, con.normal);
        real mu_static = 0.5 * (mat1.mu_static + mat2.mu_static);
        if (con.tangent_lambda < mu_static * con.normal_lambda) {
            project_positions(rb1, rb2, dp_t, 0, con.r1, con.r2, con.tangent_lambda);
        }
    }
}

void project_velocities(PBDRigidBody& rb1, PBDRigidBody& rb2, glm::rvec3 dv, glm::rvec3 r1, glm::rvec3 r2) {
    real c = glm::length(dv);
    if (c <= glm::epsilon<real>()) return;
    glm::rvec3 n = dv / c;
    real w1 = rb1.inv_mass + glmx::quadratic_form(rb1.inv_inertia, glm::cross(r1, n));
    real w2 = rb2.inv_mass + glmx::quadratic_form(rb2.inv_inertia, glm::cross(r2, n));
    real w_tot = int(rb1.is_dynamic) * w1 + int(rb2.is_dynamic) * w2;
    glm::rvec3 p = dv / w_tot;
    if (rb1.is_dynamic) {
        rb1.vel += rb1.inv_mass * p;
        rb1.angvel += rb1.inv_inertia * glm::cross(r1, p);
    }
    else if (rb2.is_dynamic) {
        rb2.vel -= rb2.inv_mass * p;
        rb2.angvel -= rb2.inv_inertia * glm::cross(r2, p);
    }
}

void project_angular_velocities(PBDRigidBody& rb1, PBDRigidBody& rb2, glm::rvec3 dw) {
    // TODO
}

void PBDWorld::solve_velocities(real h) {
    // TODO: Apply joint damping
    for (auto& con : constraints) {
        switch(con.type) {
            case PBDConstraintType::RevoluteJoint: {
                auto& rev_con = con.revolute_joint;
                auto& rb1 = *rigid_bodies.get(rev_con.rb_id1);
                auto& rb2 = *rigid_bodies.get(rev_con.rb_id2);
                glm::rvec3 dw = (rb2.angvel - rb1.angvel) * glm::min(rev_con.damping * h, real(1));
                project_angular_velocities(rb1, rb2, dw);
            } break;
            case PBDConstraintType::SphericalJoint: {
                auto& sph_con = con.spherical_joint;
                auto& rb1 = *rigid_bodies.get(sph_con.rb_id1);
                auto& rb2 = *rigid_bodies.get(sph_con.rb_id2);
                glm::rvec3 dw = (rb2.angvel - rb1.angvel) * glm::min(sph_con.damping * h, real(1));
                project_angular_velocities(rb1, rb2, dw);
            } break;
        }
    }

    // Apply contact forces
    for (auto& con : rb_rb_contact_constraints) {
        auto& rb1 = *rigid_bodies.get(con.rb_id1);
        auto& rb2 = *rigid_bodies.get(con.rb_id2);
        auto& mat1 = *materials.get(rb1.mat_id);
        auto& mat2 = *materials.get(rb2.mat_id);
        real mu_dynamic = 0.5 * (mat1.mu_dynamic + mat2.mu_dynamic);
        real restitution = 0.5 * (mat1.restitution + mat2.restitution);
        glm::rvec3 v = (rb1.vel + glm::cross(rb1.angvel, con.r1)) - (rb2.vel + glm::cross(rb2.angvel, con.r2));
        real v_n = glm::dot(con.normal, v);
        glm::rvec3 v_t = v - v_n * con.normal;
        real v_t_len = glm::length(v_t);
        if (v_t_len > glm::epsilon<real>()) {
            glm::rvec3 dv = -glm::min(mu_dynamic * con.normal_lambda / h, v_t_len) * v_t / v_t_len;
            project_velocities(rb1, rb2, dv, con.r1, con.r2);
        }
        glm::rvec3 v_next = (rb1.vel + glm::cross(rb1.angvel, con.r1)) - (rb2.vel + glm::cross(rb2.angvel, con.r2));
        real v_n_next = glm::dot(con.normal, v_next);
        if (v_n_next < 2*glm::length(gravity)*h) {
            restitution = 0;
        }
        glm::rvec3 dv = con.normal * (-v_n_next + glm::max(-restitution * v_n, real(0)));
        project_velocities(rb1, rb2, dv, con.r1, con.r2);
    }

}


}
