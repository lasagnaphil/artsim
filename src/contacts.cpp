//
// Created by lasagnaphil on 1/26/21.
//

#include "artsim/contacts.h"

#include "artsim/dynamics.h"

#include <Eigen/Dense>

using namespace glm;
using namespace glmx;

namespace artsim {
/*
 * TODO for contact solver:
 * - PGS:
 *      - Use nesterov momentum for better convergence (See https://apps.dtic.mil/dtic/tr/fulltext/u2/1081106.pdf)
 * - Bisection:
 *      - Still seems to be unstable. Investigate why.
 */

inline real compute_r(real theta, glm::tvec3<real> Minv_r3, real c_z, real mu) {
    return -c_z / (Minv_r3.z / mu + Minv_r3.x * cos(theta) + Minv_r3.y * sin(theta));
}

inline real bisection_gradient(const tsmat3x3<real>& Minv, tvec3<real> c, tvec3<real> lambda, real mu) {
    glm::tvec3<real> Minv_r3 = tvec3<real>(Minv.zx, Minv.yz, Minv.zz);
    glm::tvec3<real> eta = glm::cross(Minv_r3, glm::tvec3<real>(lambda.x, lambda.y, -mu*mu*lambda.z));
    return glm::dot(Minv * lambda + c, eta);
}

static glm::tvec3<real> contact_bisection_solver(tvec3<real> lambda_v0, const tsmat3x3<real>& Minv, tvec3<real> c, real mu) {
    const real gamma = 1e-4;
    real theta = glm::atan(lambda_v0.y, lambda_v0.x);
    tvec3<real> Minv_r3 = tvec3<real>(Minv.zx, Minv.yz, Minv.zz);
    real r = compute_r(theta, Minv_r3, c.z, mu);
    real lambda_z = r / mu;
    tvec3<real> lambda = tvec3<real>(r*cos(theta), r*sin(theta), lambda_z);
    real D0 = bisection_gradient(Minv, c, lambda, mu);

    real theta_p_delta = glm::asin(mu*lambda_v0.z / glm::sqrt(lambda_v0.x*lambda_v0.x + lambda_v0.y*lambda_v0.y));
    real theta_p;
    if (D0 >= 0) {
        theta_p = theta - pi<real>()/2 + theta_p_delta;
    }
    else {
        theta_p = theta + pi<real>()/2 - theta_p_delta;
    }

    int iter = 0;
    tvec3<real> lambda_b;
    real theta_orig = theta;
    real theta_p_orig = theta_p;
    do {
        real theta_b = 0.5 * (theta + theta_p);
        real r_b = compute_r(theta_b, Minv_r3, c.z, mu);
        real lambda_z_b = r_b / mu;
        lambda_b = vec3(r_b*cos(theta_b), r_b*sin(theta_b), lambda_z_b);
        real grad = bisection_gradient(Minv, c, lambda_b, mu);
        if (grad * D0 > 0) { theta_p = theta_b; }
        else { theta = theta_b; }
        iter++;
        if (iter == 20) {
            output_log("Bisection solver: bisection cannot find root!\n");
            exit(EXIT_FAILURE);
        }
    }
    while (abs(theta - theta_p) >= gamma);
    // output_log("Bisection solver: bisection finished in %d iters\n", iter);

    return lambda_b;
}

static tvec3<real> contact_projection_solver(tvec3<real> lambda, const tsmat3x3<real>& Minv, tvec3<real> c, real mu) {
    const real alpha = 1.0f;
    real r_z = alpha / Minv.zz;
    real r_t = alpha / max(Minv.xx, Minv.yy);
    tvec3<real> v = c + Minv*lambda;
    real lambda_z = max(real(0), lambda.z - r_z*v.z);
    // TODO: Do real euclidean projection on conic disk
    tvec2<real> lambda_t = tvec2<real>(lambda.x - r_t*v.x, lambda.y - r_t*v.y);
    real lambda_t_len = length(lambda_t);
    if (lambda_t_len > mu*lambda_z) {
        lambda_t = mu*lambda_z*normalize(lambda_t);
    }
    return tvec3<real>(lambda_t.x, lambda_t.y, lambda_z);
}

static std::tuple<glm::tvec3<real>, real, bool> contact_ncp_solver(tvec3<real> lambda_v0,
                                                                   const tsmat3x3<real>& Minv, tvec3<real> c, real mu, real r) {
    // Initial value for lambda
    tvec3<real> lambda = lambda_v0;
    tvec3<real> lambda_prev = lambda;

    // Damping parameter for Newton method
    const real alpha = real(0.75);

    // Newton-Raphson method with NCP formulation
    real ncp_error_sq;
    real ncp_error_sq_prev = 1e8;
    real w;

    bool success = true;

    for (int i = 0; i < 16; i++) {
        tvec3<real> v = c + Minv*lambda;

        // Calculate Jacobian of the current system
        tsmat3x3<real> J;
        {
            real a = glm::sqrt(v.x*v.x + v.y*v.y);
            real b = r*(mu*lambda.z - glm::sqrt(lambda.x*lambda.x + lambda.y*lambda.y));
            real d = glm::sqrt(a*a + b*b);
            w = (d - b) / (a + r*mu*lambda.z - d);

            J.xx = Minv.xx + w;
            J.xy = Minv.xy;
            J.yy = Minv.yy + w;
        }
        {
            real d = glm::sqrt(v.z*v.z + r*r*lambda.z*lambda.z);
            real dphi_dvn = real(1) - v.z/d;

            J.zx = dphi_dvn * Minv.zx;
            J.yz = dphi_dvn * Minv.yz;
            J.zz = r*(real(1) - r*lambda.z/d) + dphi_dvn * Minv.zz;
        }

        // NCP functions
        tvec3<real> phi;
        phi.x = v.x + w * lambda.x;
        phi.y = v.y + w * lambda.y;
        phi.z = glm::sqrt(v.z*v.z + lambda.z*lambda.z) - v.z - lambda.z;

        // Newton step
        lambda -= alpha * (inverse(J) * phi);

        phi.x = v.x + w * lambda.x;
        phi.y = v.y + w * lambda.y;
        phi.z = glm::sqrt(v.z*v.z + lambda.z*lambda.z) - v.z - lambda.z;

        ncp_error_sq = glm::length2(phi);
        output_log("NCP error: %f\n", ncp_error_sq);
        if (ncp_error_sq > ncp_error_sq_prev) {
            // Newton method failed
            lambda = lambda_prev;
            ncp_error_sq = ncp_error_sq_prev;
            success = false;
            break;
        }

        ncp_error_sq_prev = ncp_error_sq;
        lambda_prev = lambda;

        if (ncp_error_sq <= real(1e-8)) {
            // Newton method finished
            break;
        }
    }

    return {lambda, ncp_error_sq, success};
}


void solve_collision(ContactSolverType type, uint32_t max_iters,
                     const ArticulatedBody& art, const MaterialDB& material_db, glm::tvec3<real> gravity, real dt,
                     const real* q, const real* u, const real* udot_orig, const tscrew<real>* f_ext, const real* tau,
                     const ContactPoint* contact_points, uint32_t num_contact_points,
                     glm::tvec3<real>* out_lambda, real* out_contact_forces) {

    using namespace Eigen;

    int num_vel_dofs = art.get_num_vel_dofs();
    int num_joints = art.get_num_joints();

    Eigen::Matrix<real, Dynamic, 1> u_bar(num_vel_dofs);
    for (int i = 0; i < num_vel_dofs; i++) {
        u_bar[i] = u[i] + udot_orig[i] * dt;
    }

    std::vector<tscrew<real>> S(num_vel_dofs);
    calc_S(art, q, OUT S.data());

    std::vector<ttransform<real>> T_link_global(num_joints), T_joint_global(num_joints);
    calc_transforms(art, q, OUT T_link_global.data(), OUT T_joint_global.data());

    Eigen::Matrix<real, Dynamic, Dynamic> Jc_T(num_vel_dofs, 3*num_contact_points);
    std::vector<tscrew<real>> J_local(num_vel_dofs);

    std::vector<tvec3<real>> c(num_contact_points);
    std::vector<tvec3<real>> lambda(num_contact_points, tvec3<real>(0));

    for (int c = 0; c < num_contact_points; c++) {
        const auto& cp = contact_points[c];
        if (cp.body1_id.is_articulation() && cp.body2_id == BodyId::from_ground()) {
            auto tangent_u = Ez<real>();
            auto tangent_v = glm::cross(cp.normal, tangent_u);
            auto contact_T = ttransform<real>(cp.pos, glm::tmat3x3<real>(tangent_u, tangent_v, cp.normal));
            auto [art_id, art_link_idx] = cp.body1_id.get_articulation_id();
            calculate_jacobian_for_local_frame(art, art_link_idx,
                                               contact_T, T_joint_global.data(), S.data(),
                                               OUT J_local.data());
            for (int i = 0; i < num_vel_dofs; i++) {
                Jc_T(i, 3*c + 0) = J_local[i].v[0];
                Jc_T(i, 3*c + 1) = J_local[i].v[1];
                Jc_T(i, 3*c + 2) = J_local[i].v[2];
            }
        }
    }

    Eigen::Matrix<real, Dynamic, 1> tau_star = Jc_T.transpose() * u_bar;

    const real beta = 0.01;
    const real slop = 5e-5;

    for (int i = 0; i < num_contact_points; i++) {
        c[i] = make_vec3<real>(tau_star.data() + 3*i);
        c[i].z -= beta/dt*glm::max<real>(contact_points[i].depth - slop, 0);
    }

    dynmat<tsmat3x3<real>> M_contact_inv(num_contact_points, num_contact_points);

    Eigen::Matrix<real, Dynamic, Dynamic> Minv_Jc_T(num_vel_dofs, 3*num_contact_points);
    std::vector<real> zero_vec(num_vel_dofs, 0);

    dynmat_view<real> Minv_Jc_T_view(Minv_Jc_T.data(), 3*num_contact_points, num_vel_dofs);
    dynmat_view<real> Jc_T_view(Jc_T.data(), 3*num_contact_points, num_vel_dofs);

    multiply_inverse_mass_matrix(art, dt, q, Jc_T_view, OUT Minv_Jc_T_view);

    /*
    for (int k = 0; k < 3*num_contact_points; k++) {
        featherstone_forward_dynamics(art, glm::tvec3<real>(0), dt, nullptr, q, zero_vec.data(), Jc_T.data() + k*num_vel_dofs,
                                      OUT Minv_Jc_T.data() + k*num_vel_dofs);
    }
     */

    for (int k = 0; k < num_contact_points; k++) {
        Eigen::Matrix<real, Dynamic, 3> Minv_Jck_T = Minv_Jc_T.middleCols<3>(3*k);
        for (int i = 0; i < num_contact_points; i++) {
            Eigen::Matrix<real, 3, Dynamic> Jci = Jc_T.middleCols<3>(3*i).transpose();
            Eigen::Matrix<real, 3, 3> M_contact_inv_eigen = Jci * Minv_Jck_T;
            M_contact_inv(i, k) = tsmat3x3<real>(
                    M_contact_inv_eigen(0, 0),
                    M_contact_inv_eigen(1, 1),
                    M_contact_inv_eigen(2, 2),
                    M_contact_inv_eigen(1, 2),
                    M_contact_inv_eigen(2, 0),
                    M_contact_inv_eigen(0, 1));
        }
    }

    iterative_contact_solver(type, max_iters, dt, num_contact_points, M_contact_inv, INOUT c.data(), INOUT
                             lambda.data());

    Eigen::Matrix<real, Dynamic, 1> lambda_vec = Map<Eigen::Matrix<real, Dynamic, 1>>((real*)lambda.data(), 3*num_contact_points);
    Eigen::Matrix<real, Dynamic, 1> contact_forces = Jc_T * lambda_vec / dt;

    if (out_lambda) {
        std::memcpy(out_lambda, lambda.data(), sizeof(tvec3<real>) * num_contact_points);
    }
    if (out_contact_forces) {
        std::memcpy(out_contact_forces, contact_forces.data(), sizeof(real) * num_vel_dofs);
    }
}

void iterative_contact_solver(
        ContactSolverType type, uint32_t max_iters, real dt,
        uint32_t num_contact_points,
        const dynmat<tsmat3x3<real>>& M_contact_inv,
        INOUT tvec3<real>* c, INOUT tvec3<real>* lambda) {

    real alpha_min, gamma, lambda_sq_tol, ncp_error_sq_tol;
    real alpha, total_ncp_error_sq;

    // TODO: Make friction coefficient changable
    const real mu = 1.0;

    switch (type) {
        case ContactSolverType::PGS:
            alpha = 0.75;
            alpha_min = 0.75;
            gamma = 1.0;
            lambda_sq_tol = 1e-6;
            break;
        case ContactSolverType::Bisection:
            alpha = 1.0;
            alpha_min = 0.7;
            gamma = 0.99;
            lambda_sq_tol = 1e-6;
            break;
        case ContactSolverType::NCP:
            alpha = 1.0;
            alpha_min = 1.0;
            gamma = 1.0;
            lambda_sq_tol = 1e-6;
            ncp_error_sq_tol = 1e-6;
            total_ncp_error_sq = 0;
            break;
    }

    real lambda_norm2;
    std::vector<tvec3<real>> lambda_old(num_contact_points);
    int iter;
    for (iter = 0; iter < max_iters; iter++) {
        std::copy(lambda, lambda + num_contact_points, lambda_old.begin());

        for (int i = 0; i < num_contact_points; i++) {
            if (c[i].z > 0) {
                lambda[i] = (1 - alpha)*lambda[i];
            }
            else {
                tsmat3x3<real> M_inv_ii = M_contact_inv(i, i);
                tvec3<real> lambda_v0 = -(inverse(M_inv_ii) * c[i]);
                if (mu*mu * lambda_v0.z*lambda_v0.z >= lambda_v0.x*lambda_v0.x + lambda_v0.y*lambda_v0.y) {
                    lambda[i] = alpha * lambda_v0 + (1 - alpha) * lambda[i];
                }
                else {
                    tvec3<real> lambda_star;
                    switch (type) {
                        case ContactSolverType::PGS: {
                            lambda_star = contact_projection_solver(lambda[i], M_inv_ii, c[i], mu);
                        } break;
                        case ContactSolverType::Bisection: {
                            lambda_star = contact_bisection_solver(lambda_v0, M_inv_ii, c[i], mu);
                        } break;
                        case ContactSolverType::NCP: {
                            real ncp_error_sq, success;
                            std::tie(lambda_star, ncp_error_sq, success) = contact_ncp_solver(lambda_v0, M_inv_ii, c[i], mu, dt);
                        } break;
                    }
                    lambda[i] = alpha * lambda_star + (1 - alpha) * lambda[i];
                }
            }

            for (int ip = 0; ip < num_contact_points; ip++) {
                if (i == ip) continue;
                tsmat3x3<real> M_ip_i_inv = M_contact_inv(ip, i);
                c[ip] += M_ip_i_inv*(lambda[i] - lambda_old[i]);
            }
        }
        alpha = alpha_min + gamma * (alpha - alpha_min);

        lambda_norm2 = 0.0;
        for (int i = 0; i < num_contact_points; i++) {
            lambda_norm2 += length2(lambda[i] - lambda_old[i]);
        }
        if (lambda_norm2 < lambda_sq_tol) {
            iter++; break;
        }
    }
    if (iter == max_iters) {
        output_log("Contact solver did not converge! (error = %f)\n", sqrt(lambda_norm2));
    }
    else {
        output_log("Contact solver converged in %d iters\n", iter);
    }

}

void
euler_step_with_collision(ContactSolverType type, uint32_t max_iters,
                          const ArticulatedBody& art, const MaterialDB& material_db, glm::tvec3<real> gravity, real dt,
                          const tscrew<real>* f_ext, const real* tau, const ContactPoint* contact_points,
                          uint32_t num_contact_points, real* q, real* u, real* udot, glm::tvec3<real>* lambda) {
    int num_vel_dofs = art.get_num_vel_dofs();
    int num_joints = art.get_num_joints();

    std::vector<real> udot_bar(num_vel_dofs);
    featherstone_forward_dynamics(art, gravity, dt, f_ext, q, u, tau, OUT udot_bar.data());

    if (num_contact_points == 0) {
        integrate_implicit_euler(art, dt, udot_bar.data(), INOUT q, INOUT u);
    }
    else {
        auto t1 = std::chrono::high_resolution_clock::now();
        std::vector<real> tau_contact(num_vel_dofs);
        std::vector<real> tau_total(num_vel_dofs);

        solve_collision(type, max_iters,
                        art, material_db, gravity, dt,
                        q, u, udot_bar.data(),
                        f_ext, tau,
                        contact_points, num_contact_points,
                        OUT lambda, OUT tau_contact.data());

        auto t2 = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1);
        printf("Contact solver: %lld ns\n", duration.count());

        for (int i = 0; i < num_vel_dofs; i++) {
            tau_total[i] = tau[i] + tau_contact[i];
            // printf("%f ", tau_contact[i]);
        }
        // printf("\n");
        featherstone_forward_dynamics(art, gravity, dt, f_ext, q, u, tau_total.data(), OUT udot);

        integrate_implicit_euler(art, dt, udot, INOUT q, INOUT u);
    }
}

std::vector<ContactPoint> get_contact_points_bullet(btCollisionWorld* bt_world) {
    auto dispatcher = bt_world->getDispatcher();
    btPersistentManifold** manifolds = dispatcher->getInternalManifoldPointer();
    int num_manifolds = dispatcher->getNumManifolds();

    std::vector<ContactPoint> contact_points;
    for (int i = 0; i < num_manifolds; i++) {
        btPersistentManifold* manifold = manifolds[i];
        int num_contacts = manifold->getNumContacts();
        if (num_contacts == 0) continue;

        const btCollisionObject* body1 = manifold->getBody0();
        const btCollisionObject* body2 = manifold->getBody1();
        BodyId body1_id, body2_id;
        body1_id.index = body1->getUserIndex();
        body1_id.generation = body1->getUserIndex2();
        body2_id.index = body2->getUserIndex();
        body2_id.generation = body2->getUserIndex2();
        if (body1_id.index < body2_id.index) std::swap(body1_id, body2_id);
        auto contact_pos = tvec3<real>(0);
        auto contact_normal = tvec3<real>(0);
        real contact_depth = 0, contact_area = 0;
        for (int j = 0; j < num_contacts; j++) {
            auto& pt = manifold->getContactPoint(j);
            contact_pos += glmconv(pt.getPositionWorldOnB());
            contact_normal += glmconv(pt.m_normalWorldOnB);
            contact_depth -= pt.getDistance();
        }
        if (contact_depth < 0) continue;
        contact_pos /= num_contacts;
        contact_normal = glm::normalize(contact_normal);
        contact_depth /= num_contacts;
        if (num_contacts == 3) {
            glm::tvec3<real> v1 = glmconv(manifold->getContactPoint(0).getPositionWorldOnB());
            glm::tvec3<real> v2 = glmconv(manifold->getContactPoint(1).getPositionWorldOnB());
            glm::tvec3<real> v3 = glmconv(manifold->getContactPoint(2).getPositionWorldOnB());
            contact_area = 0.5f * glm::length(glm::cross(v2 - v1, v3 - v1));
        }
        else if (num_contacts == 4) {
            glm::tvec3<real> v1 = glmconv(manifold->getContactPoint(0).getPositionWorldOnB());
            glm::tvec3<real> v2 = glmconv(manifold->getContactPoint(1).getPositionWorldOnB());
            glm::tvec3<real> v3 = glmconv(manifold->getContactPoint(2).getPositionWorldOnB());
            glm::tvec3<real> v4 = glmconv(manifold->getContactPoint(3).getPositionWorldOnB());
            contact_area = 0.5f * glm::length(glm::cross(v2 - v1, v3 - v1));
            contact_area += 0.5f * glm::length(glm::cross(v3 - v1, v4 - v1));
        }

        ContactPoint cp;
        cp.bt_manifold = manifold;
        cp.pos = contact_pos;
        cp.normal = contact_normal;
        cp.depth = contact_depth;
        cp.area = contact_area;
        cp.body1_id = body1_id;
        cp.body2_id = body2_id;
        contact_points.push_back(cp);
    }
    return contact_points;
}

std::vector<ContactPoint>
contact_points_between_art_links_and_ground(const ArticulatedBody& art, const Id<ArticulatedBody> art_id,
                                            const uint32_t* link_indices, uint32_t link_indices_count,
                                            const ttransform<real>* link_global_trans) {

    std::vector<ContactPoint> contact_points;

    const float epsilon = 1e-7f;
    for (uint32_t li = 0; li < link_indices_count; li++) {
        std::vector<glm::tvec3<real>> cpos;
        uint32_t i = link_indices[li];
        switch (art.links[i].shape.type) {
            case artsim::Shape::Type::Box: {
                glm::tvec3<real> ext = real(0.5) * art.links[i].shape.box.size;
                glm::tvec3<real> p = link_global_trans[i].v;
                if (p.y*p.y > ext.x*ext.x + ext.y*ext.y + ext.z*ext.z) {
                    // early bailout for boxes that definitely doesn't collide with ground
                    break;
                }
                p = link_global_trans[i].v + link_global_trans[i].R * glm::tvec3<real>(-ext.x, -ext.y, -ext.z);
                if (p.y <= epsilon) {
                    cpos.push_back(p);
                }
                p = link_global_trans[i].v + link_global_trans[i].R * glm::tvec3<real>(-ext.x, -ext.y,  ext.z);
                if (p.y <= epsilon) {
                    cpos.push_back(p);
                }
                p = link_global_trans[i].v + link_global_trans[i].R * glm::tvec3<real>(-ext.x,  ext.y, -ext.z);
                if (p.y <= epsilon) {
                    cpos.push_back(p);
                }
                p = link_global_trans[i].v + link_global_trans[i].R * glm::tvec3<real>(-ext.x,  ext.y,  ext.z);
                if (p.y <= epsilon) {
                    cpos.push_back(p);
                }
                p = link_global_trans[i].v + link_global_trans[i].R * glm::tvec3<real>( ext.x, -ext.y, -ext.z);
                if (p.y <= epsilon) {
                    cpos.push_back(p);
                }
                p = link_global_trans[i].v + link_global_trans[i].R * glm::tvec3<real>( ext.x, -ext.y,  ext.z);
                if (p.y <= epsilon) {
                    cpos.push_back(p);
                }
                p = link_global_trans[i].v + link_global_trans[i].R * glm::tvec3<real>( ext.x,  ext.y, -ext.z);
                if (p.y <= epsilon) {
                    cpos.push_back(p);
                }
                p = link_global_trans[i].v + link_global_trans[i].R * glm::tvec3<real>( ext.x,  ext.y,  ext.z);
                if (p.y <= epsilon) {
                    cpos.push_back(p);
                }
#if 1
                if (!cpos.empty()) {
                    auto cpos_avg = glm::tvec3<real>(0);
                    for (auto& pos : cpos) {
                        cpos_avg += pos;
                    }
                    cpos_avg /= cpos.size();
                    ContactPoint cp;
                    cp.bt_manifold = nullptr;
                    cp.pos = glm::tvec3<real>(cpos_avg.x, 0, cpos_avg.z);
                    cp.normal = Ey<real>();
                    cp.depth = -cpos_avg.y;
                    if (cpos.size() == 3) {
                        cp.area = 0.5f * glm::length(glm::cross(cpos[1] - cpos[0], cpos[2] - cpos[0]));
                    }
                    else if (cpos.size() == 4) {
                        cp.area = 0.5f * glm::length(glm::cross(cpos[1] - cpos[0], cpos[2] - cpos[0]));
                        cp.area += 0.5f * glm::length(glm::cross(cpos[2] - cpos[0], cpos[3] - cpos[0]));
                    }
                    cp.body1_id = BodyId::from_articulation_link(art_id, i);
                    cp.body2_id = BodyId::from_ground();
                    contact_points.push_back(cp);
                }
#else
                for (auto& pos : cpos) {
                    contact_points.emplace_back(
                            glm::vec3(pos.x, 0, pos.z), Ey<real>(), Ez<real>(), -pos.y,
                            BodyId::from_articulation_link(art_id, i),
                            BodyId::from_rigid_body(Id<RigidBody>::null()));
                }
#endif
            } break;
            case artsim::Shape::Type::Sphere: {
                glm::tvec3<real> p = link_global_trans[i].v;
                real r = art.links[i].shape.sphere.radius;
                real d = p.y - r;
                if (d <= 0.0f) {
                    ContactPoint cp;
                    cp.bt_manifold = nullptr;
                    cp.pos = glm::vec3(p.x, 0, p.z);
                    cp.normal = Ey<real>();
                    cp.depth = -d;
                    cp.area = M_PI * (r*r - (r - p.y)*(r - p.y));
                    cp.body1_id = BodyId::from_articulation_link(art_id, i);
                    cp.body2_id = BodyId::from_ground();
                    contact_points.push_back(cp);
                }
            } break;
        }
    }

    return contact_points;
}
}
