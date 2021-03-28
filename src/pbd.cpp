//
// Created by lasagnaphil on 3/27/21.
//

#include <artsim/pbd.h>

namespace artsim {

PBDWorld::PBDWorld() {
    bt_collision_config = std::make_unique<btDefaultCollisionConfiguration>();
    bt_dispatcher = std::make_unique<btCollisionDispatcher>(bt_collision_config.get());
    bt_broadphase = std::make_unique<btDbvtBroadphase>();
    bt_world = std::make_unique<btCollisionWorld>(bt_dispatcher.get(), bt_broadphase.get(), bt_collision_config.get());
}

void PBDWorld::reset() {
    for (auto& rb : rigid_bodies) {
        auto* col_obj = bt_collision_objects.get(rb.col_obj);
        bt_world->removeCollisionObject(col_obj);
    }
    bt_collision_objects.clear();
    bt_collision_shapes.clear();
    rigid_bodies.clear();
    constraints.clear();
}

Id<PBDRigidBody> PBDWorld::make_cube(glm::rvec3 size, artsim::real mass,
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
    rb.col_shape = bt_collision_shapes.make_box(real(0.5) * s);
    btCollisionShape* box_shape = bt_collision_shapes.get(rb.col_shape);
    rb.col_obj = bt_collision_objects.insert(btCollisionObject());
    btCollisionObject* collision_obj = bt_collision_objects.get(rb.col_obj);
    rb.is_dynamic = true;
    rb.pos = pos;
    rb.rot = rot;
    rb.vel = vel;
    rb.angvel = angvel;
    rb.f_ext = f_ext;
    rb.tau_ext = tau_ext;
    Id<PBDRigidBody> id = rigid_bodies.insert(rb);

    auto [user_id1, user_id2] = id.to_int32s();
    collision_obj->setCollisionShape(box_shape);
    collision_obj->setWorldTransform(btconv(glmx::rquat_transform(rb.pos, rb.rot)));
    collision_obj->setUserIndex(user_id1);
    collision_obj->setUserIndex(user_id2);
    bt_world->addCollisionObject(collision_obj);

    return id;
}

/*
Id<PBDConstraint>
PBDWorld::make_positional_constraint(Id<PBDRigidBody> rb_id, glm::rvec3 offset, glm::rvec3 pos) {
    if (!rigid_bodies.is_valid(rb_id)) return {};
    PBDConstraint cons;
    cons.type = PBDConstraintType::FixedRevoluteJoint;
    cons.fix_position.rb_id = rb_id;
    cons.fix_position.offset = offset;
    cons.fix_position.pos = pos;
    cons.fix_position.lambda = 0;
    return constraints.insert(cons);
}
 */

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
    cons.revolute_joint.pos_lambda = 0;
    cons.revolute_joint.rot_lambda = 0;
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
    cons.spherical_joint.pos_lambda = 0;
    cons.spherical_joint.swing_rot_lambda = 0;
    cons.spherical_joint.twist_rot_lambda = 0;
    return constraints.insert(cons);
}

void PBDWorld::simulate(real dt, int num_substeps) {
    PBDRigidBody* rbs = rigid_bodies.get_items_buf();
    int num_rbs = rigid_bodies.size();

    reset_lambdas();

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
                rb.rot += glm::rquat(0, 0.5*h*rb.angvel);
                rb.rot = glm::normalize(rb.rot);
            }
        }
        solve_positions(h);
        for (int rb_idx = 0; rb_idx < num_rbs; rb_idx++) {
            PBDRigidBody& rb = rbs[rb_idx];
            if (rb.is_dynamic) {
                rb.vel = (rb.pos - rb.prev_pos) / h;
                glm::rquat dq = rb.rot * glm::inverse(rb.prev_rot);
                rb.angvel = (2.0/h) * glm::rvec3(dq.x, dq.y, dq.z);
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
    auto dispatcher = bt_world->getDispatcher();
    btPersistentManifold** manifolds = dispatcher->getInternalManifoldPointer();
    int num_manifolds = dispatcher->getNumManifolds();

    for (int i = 0; i < num_manifolds; i++) {
        btPersistentManifold* manifold = manifolds[i];
        int num_contacts = manifold->getNumContacts();
        if (num_contacts == 0) continue;

        const btCollisionObject* body1 = manifold->getBody0();
        const btCollisionObject* body2 = manifold->getBody1();
        Id<PBDRigidBody> body1_id = Id<PBDRigidBody>::from_int32s(body1->getUserIndex(), body1->getUserIndex2());
        Id<PBDRigidBody> body2_id = Id<PBDRigidBody>::from_int32s(body2->getUserIndex(), body2->getUserIndex2());
        if (body1_id.index > body2_id.index) std::swap(body1_id, body2_id);
        for (int j = 0; j < num_contacts; j++) {
            auto& pt = manifold->getContactPoint(j);
            // TODO: create collision constraints
        }
    }
}

void restrict_positions(PBDRigidBody& rb,
                        real alpha, glm::rvec3 r, glm::rvec3 target_pos, real& lambda) {

    glm::rvec3 dx = target_pos - rb.pos - r;
    real c = glm::length(dx);
    if (c <= glm::epsilon<real>()) return;
    glm::rvec3 n = dx / c;
    real w = rb.inv_mass + glmx::quadratic_form(rb.inv_inertia, glm::cross(r, n));
    real dlambda = (-c - alpha * lambda) / (w + alpha);
    lambda += dlambda;

    glm::rvec3 p = dlambda * n;
    rb.pos -= rb.inv_mass * p;
    rb.rot -= 0.5 * (glm::rquat(0, rb.inv_inertia * glm::cross(r, p)) * rb.rot);
    rb.rot = glm::normalize(rb.rot);

}

void restrict_positions(PBDRigidBody& rb1, PBDRigidBody& rb2,
                        real alpha, glm::rvec3 r1, glm::rvec3 r2, real& lambda) {
    glm::rvec3 x_com = (rb1.mass * rb1.pos + rb2.mass * rb2.pos) / (rb1.mass + rb2.mass);
    glm::rvec3 dx = (rb2.pos + r2) - (rb1.pos + r1);
    real c = glm::length(dx);
    if (c <= glm::epsilon<real>()) return;
    glm::rvec3 n = dx / c;
    real w1 = rb1.inv_mass + glmx::quadratic_form(rb1.inv_inertia, glm::cross(r1, n));
    real w2 = rb2.inv_mass + glmx::quadratic_form(rb2.inv_inertia, glm::cross(r2, n));
    real dlambda = (-c - alpha * lambda) / (w1 + w2 + alpha);
    lambda += dlambda;

    glm::rvec3 p = dlambda * n;
    rb1.pos -= rb1.inv_mass * p;
    rb2.pos += rb2.inv_mass * p;
    rb1.rot -= 0.5 * (glm::rquat(0, rb1.inv_inertia * glm::cross(r1, p)) * rb1.rot);
    rb2.rot += 0.5 * (glm::rquat(0, rb2.inv_inertia * glm::cross(r2, p)) * rb2.rot);
    rb1.rot = glm::normalize(rb1.rot);
    rb2.rot = glm::normalize(rb2.rot);
}

void restrict_rotations(PBDRigidBody& rb,
                        real alpha, glm::rquat target_rot, real& lambda) {

    glm::rvec3 dq = glmx::log(target_rot * glm::inverse(rb.rot));
    real theta = glm::length(dq);
    if (theta <= glm::epsilon<real>()) return;
    glm::rvec3 n = dq / theta;
    glm::rvec3 n_rel = glm::inverse(rb.rot) * n;
    real w = glmx::quadratic_form(rb.inv_inertia, n_rel);
    real dlambda = (-theta - alpha * lambda) / (w + alpha);
    lambda += dlambda;

    glm::rvec3 p = dlambda * n_rel;
    rb.rot -= 0.5 * (glm::rquat(0, rb.inv_inertia * p) * rb.rot);
    rb.rot = glm::normalize(rb.rot);
}

void restrict_rotations(PBDRigidBody& rb1, PBDRigidBody& rb2, glm::rvec3 dq,
                        real alpha, real& lambda) {
    real theta = glm::length(dq);
    glm::rvec3 n = dq / theta;
    glm::rvec3 n_rel = glm::inverse(rb1.rot) * n;
    real w1 = glmx::quadratic_form(rb1.inv_inertia, n_rel);
    real w2 = glmx::quadratic_form(rb2.inv_inertia, n_rel);
    real dlambda = (-theta - alpha * lambda) / (w1 + w2 + alpha);
    lambda += dlambda;

    glm::rvec3 p = dlambda * n_rel;
    rb1.rot -= 0.5 * (glm::rquat(0, rb1.inv_inertia * p) * rb1.rot);
    rb2.rot += 0.5 * (glm::rquat(0, rb2.inv_inertia * p) * rb2.rot);
    rb1.rot = glm::normalize(rb1.rot);
    rb2.rot = glm::normalize(rb2.rot);
}

void restrict_rotations(PBDRigidBody& rb1, PBDRigidBody& rb2, real alpha, real& lambda) {
    glm::rvec3 dq = glmx::log(rb2.rot * glm::inverse(rb1.rot));
    restrict_rotations(rb1, rb2, dq, alpha, lambda);
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
    for (auto& con : constraints) {
        real alpha = con.compliance / h_sq;
        switch (con.type) {
            /*
            case PBDConstraintType::FixedRevoluteJoint: {
                auto& pos_con = con.fix_position;
                auto* rb = rigid_bodies.get(pos_con.rb_id);
                restrict_positions(*rb, alpha, pos_con.offset, pos_con.pos, pos_con.lambda);
            } break;
            case PBDConstraintType::FixedPrismaticJoint: {
                auto& rot_con = con.fix_rotation;
                auto* rb = rigid_bodies.get(rot_con.rb_id);
                restrict_rotations(*rb, alpha, rot_con.offset, rot_con.rot, rot_con.lambda);
            } break;
            */
            case PBDConstraintType::RevoluteJoint: {
                auto& rev_con = con.revolute_joint;
                auto* rb1 = rigid_bodies.get(rev_con.rb_id1);
                auto* rb2 = rigid_bodies.get(rev_con.rb_id2);
                if (rb1->is_dynamic && rb2->is_dynamic) {
                    glm::rmat3 basis1 = glm::mat3_cast(rb1->rot);
                    glm::rmat3 basis2 = glm::mat3_cast(rb2->rot);
                    restrict_positions(*rb1, *rb2,
                                       alpha, basis1 * rev_con.offset1, basis2 * rev_con.offset2, rev_con.pos_lambda);
                    glm::rvec3 dq;
                    if (limit_angle(basis1[0], basis1[1], basis2[1], rev_con.limit_min, rev_con.limit_max, dq)) {
                        restrict_rotations(*rb1, *rb2, dq, alpha, rev_con.rot_lambda);
                    }
                }
                else if (!rb1->is_dynamic && !rb2->is_dynamic) {
                    break;
                }
                else {
                    glm::rvec3 offset1 = rev_con.offset1;
                    glm::rvec3 offset2 = rev_con.offset2;
                    if (!rb1->is_dynamic) {
                        std::swap(rb1, rb2);
                        std::swap(offset1, offset2);
                    }
                    glm::rmat3 basis1 = glm::mat3_cast(rb1->rot);
                    glm::rmat3 basis2 = glm::mat3_cast(rb2->rot);
                    restrict_positions(*rb1, alpha,
                                       basis1 * offset1, rb2->pos + basis2 * offset2, rev_con.pos_lambda);
                    glm::rvec3 dq;
                    if (limit_angle(basis1[0], basis1[1], basis2[1], rev_con.limit_min, rev_con.limit_max, dq)) {
                        restrict_rotations(*rb1, alpha, dq, rev_con.rot_lambda);
                    }
                }
            } break;
            case PBDConstraintType::SphericalJoint: {
                auto& sph_con = con.spherical_joint;
                auto* rb1 = rigid_bodies.get(sph_con.rb_id1);
                auto* rb2 = rigid_bodies.get(sph_con.rb_id2);
                if (rb1->is_dynamic && rb2->is_dynamic) {
                    glm::rmat3 basis1 = glm::mat3_cast(rb1->rot);
                    glm::rmat3 basis2 = glm::mat3_cast(rb2->rot);
                    glm::rvec3 dq_swing, dq_twist;
                    restrict_positions(*rb1, *rb2, alpha, basis1*sph_con.offset1, basis2*sph_con.offset2, sph_con.pos_lambda);
                    if (limit_angle(glm::normalize(glm::cross(basis1[0], basis2[0])), basis1[1], basis2[1],
                                    sph_con.swing_limit_min, sph_con.swing_limit_max, dq_swing)) {
                        restrict_rotations(*rb1, *rb2, dq_swing, alpha, sph_con.swing_rot_lambda);
                    }
                    glm::rvec3 n = glm::normalize(basis1[0] + basis2[0]);
                    glm::rvec3 n1 = glm::normalize(basis1[1] - glm::dot(n, basis1[1])*n);
                    glm::rvec3 n2 = glm::normalize(basis2[1] - glm::dot(n, basis2[1])*n);
                    if (limit_angle(n, n1, n2, sph_con.twist_limit_min, sph_con.twist_limit_max, dq_twist)) {
                        restrict_rotations(*rb1, *rb2, dq_swing, alpha, sph_con.twist_rot_lambda);
                    }
                }
                else if (!rb1->is_dynamic && !rb2->is_dynamic) {
                    break;
                }
                else {
                    glm::rvec3 offset1 = sph_con.offset1;
                    glm::rvec3 offset2 = sph_con.offset2;
                    if (!rb1->is_dynamic) {
                        std::swap(rb1, rb2);
                        std::swap(offset1, offset2);
                    }
                    glm::rmat3 basis1 = glm::mat3_cast(rb1->rot);
                    glm::rmat3 basis2 = glm::mat3_cast(rb2->rot);
                    glm::rvec3 dq_swing, dq_twist;
                    restrict_positions(*rb1, alpha, basis1*offset1, rb2->pos + basis2*offset2, sph_con.pos_lambda);
                    if (limit_angle(glm::normalize(glm::cross(basis1[0], basis2[0])), basis1[1], basis2[1],
                                                      sph_con.swing_limit_min, sph_con.swing_limit_max, dq_swing)) {
                        restrict_rotations(*rb1, alpha, dq_swing, sph_con.swing_rot_lambda);
                    }
                    glm::rvec3 n = glm::normalize(basis1[0] + basis2[0]);
                    glm::rvec3 n1 = glm::normalize(basis1[1] - glm::dot(n, basis1[1])*n);
                    glm::rvec3 n2 = glm::normalize(basis2[1] - glm::dot(n, basis2[1])*n);
                    if (limit_angle(n, n1, n2, sph_con.twist_limit_min, sph_con.twist_limit_max, dq_twist)) {
                        restrict_rotations(*rb1, alpha, dq_swing, sph_con.twist_rot_lambda);
                    }
                }
            } break;
        }
    }
}

void PBDWorld::solve_velocities(real h) {
    // TODO
}


}
