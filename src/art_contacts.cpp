//
// Created by lasagnaphil on 1/26/21.
//

#include "artsim/art_contacts.h"

#include "artsim/art_dynamics.h"

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

inline real compute_r(real theta, const rvec3& Minv_r3, real c_z, real mu) {
    return -c_z / (Minv_r3.z / mu + Minv_r3.x * cos(theta) + Minv_r3.y * sin(theta));
}

inline real bisection_gradient(const rsmat3x3& Minv, const rvec3& c, rvec3 lambda, real mu) {
    glm::tvec3<real> Minv_r3 = tvec3<real>(Minv.zx, Minv.yz, Minv.zz);
    glm::tvec3<real> eta = glm::cross(Minv_r3, glm::tvec3<real>(lambda.x, lambda.y, -mu*mu*lambda.z));
    return glm::dot(Minv * lambda + real(0.5)*c, eta);
}

static glm::rvec3 contact_bisection_solver(
        const rvec3& lambda_v0, const rsmat3x3& Minv, const rvec3& c, real mu) {
    const real gamma = 1e-4;
    real theta = glm::atan(lambda_v0.y, lambda_v0.x);
    rvec3 Minv_r3 = tvec3<real>(Minv.zx, Minv.yz, Minv.zz);
    real r = compute_r(theta, Minv_r3, c.z, mu);
    tvec3<real> lambda = tvec3<real>(r*cos(theta), r*sin(theta), r/mu);
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
    rvec3 lambda_b;
    real theta_orig = theta;
    real theta_p_orig = theta_p;
    do {
        real theta_b = 0.5 * (theta + theta_p);
        real r_b = compute_r(theta_b, Minv_r3, c.z, mu);
        lambda_b = vec3(r_b*cos(theta_b), r_b*sin(theta_b), r_b/mu);
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

static void calc_phi_and_jacobian(tvec3<real> lambda, const tsmat3x3<real>& Minv, tvec3<real> c, real mu,
                                  OUT rvec3& phi, OUT tsmat3x3<real>& J) {

}

static std::tuple<glm::tvec3<real>, real, bool> contact_ncp_solver(const tvec3<real>& lambda_v0,
                                                                   const tsmat3x3<real>& Minv,
                                                                   const tvec3<real>& c, real mu, real r) {
    // Initial value for lambda
    tvec3<real> lambda = lambda_v0;
    tvec3<real> lambda_prev = lambda;

    // Damping parameter for Newton method
    const real alpha = real(0.75);

    // Newton-Raphson method with NCP formulation
    real ncp_error_sq;
    real ncp_error_sq_prev = std::numeric_limits<real>::max();

    bool success = true;

    for (int i = 0; i < 4; i++) {
        tvec3<real> v = c + Minv*lambda;
        real w;

        // Calculate Jacobian of the current system
        tsmat3x3<real> J;
        {
            real a = glm::sqrt(v.x*v.x + v.y*v.y);
            real b = r*(mu*lambda.z - glm::sqrt(lambda.x*lambda.x + lambda.y*lambda.y));
            real d = glm::sqrt(a*a + b*b);
            w = r*(d - b) / (a + r*mu*lambda.z - d);

            J.xx = Minv.xx + w;
            J.xy = Minv.xy;
            J.yy = Minv.yy + w;
        }

        {
            real d = glm::sqrt(v.z*v.z + r*r*lambda.z*lambda.z);
            real a = real(1) - r*lambda.z/d;
            real b = real(1) - v.z/d;

            J.zx = a * Minv.zx;
            J.yz = a * Minv.yz;
            J.zz = a * Minv.zz + b * r;
        }

        // NCP functions
        tvec3<real> phi;
        phi.x = v.x + w * lambda.x;
        phi.y = v.y + w * lambda.y;
        phi.z = glm::sqrt(v.z*v.z + lambda.z*lambda.z) - v.z - lambda.z;

        ncp_error_sq = glm::length2(phi);
        if (ncp_error_sq > ncp_error_sq_prev) {
            // Newton iteration fail: abort
            output_log("Iter %d: ncp_error=%f (fail)\n", i, sqrt(ncp_error_sq));
            lambda = lambda_prev;
            ncp_error_sq = ncp_error_sq_prev;
            success = false;
            break;
        }
        else if (ncp_error_sq <= real(1e-8)) {
            // Newton method finished
            break;
        }
        output_log("Iter %d: ncp_error=%f (success)\n", i, sqrt(ncp_error_sq));

        // Newton step
        lambda -= alpha * (inverse(J) * phi);

        ncp_error_sq_prev = ncp_error_sq;
        lambda_prev = lambda;
    }

    return {lambda, ncp_error_sq, success};
}

// TODO: handle coefficient of restitution and restitution threshold...
void solve_collision(ContactSolverType type, uint32_t max_iters,
                     const ArticulatedBodySpec& art, const Material* mat, glm::tvec3<real> gravity, real dt,
                     const real* q, const real* u, const real* udot_orig, const real* tau, const tscrew<real>* f_ext,
                     const ContactPoint* contact_points, uint32_t num_contact_points,
                     OUT glm::tvec3<real>* out_lambda) {

}

void iterative_contact_solver(
        ContactSolverType type, uint32_t max_iters, const Material* mat, real dt,
        uint32_t num_contact_points,
        const dynmat<tmat3x3<real>>& M_contact_inv,
        INOUT tvec3<real>* c, INOUT tvec3<real>* lambda) {

    real alpha_min, gamma, lambda_err_tol, ncp_error_sq_tol;
    real alpha, total_ncp_error_sq;

    switch (type) {
        case ContactSolverType::PGS:
            alpha = 1.0;
            alpha_min = 1.0;
            gamma = 1.0;
            lambda_err_tol = 1e-4;
            break;
        case ContactSolverType::Bisection:
            alpha = 1.0;
            alpha_min = 0.7;
            gamma = 0.99;
            lambda_err_tol = 1e-4;
            break;
        case ContactSolverType::NCP:
            alpha = 1.0;
            alpha_min = 1.0;
            gamma = 1.0;
            lambda_err_tol = 1e-4;
            ncp_error_sq_tol = 1e-6;
            total_ncp_error_sq = 0;
            break;
    }

    real lambda_err_sq;
    std::vector<tvec3<real>> lambda_old(num_contact_points);
    int iter;
    for (iter = 0; iter < max_iters; iter++) {
        std::copy(lambda, lambda + num_contact_points, lambda_old.begin());

        for (int i = 0; i < num_contact_points; i++) {
            if (c[i].z > 0) {
                lambda[i] = (1 - alpha)*lambda[i];
            }
            else {
                real mu = mat[i].friction;
                tsmat3x3<real> M_inv_ii = glmx::smat3_cast(M_contact_inv(i, i));
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
                            const real r = glmx::frobenius_norm(M_inv_ii);
                            real ncp_error_sq;
                            bool success;
                            std::tie(lambda_star, ncp_error_sq, success) = contact_ncp_solver(lambda_v0, M_inv_ii, c[i], mu, r);
                        } break;
                    };
                    if (glm::isnan(lambda_star[0]) || glm::isnan(lambda_star[1]) || glm::isnan(lambda_star[2])) {
                        output_log("NaN error!\n");
                    }
                    lambda[i] = alpha * lambda_star + (1 - alpha) * lambda[i];
                }
            }

            for (int ip = 0; ip < num_contact_points; ip++) {
                if (i == ip) continue;
                tmat3x3<real> M_ip_i_inv = M_contact_inv(ip, i);
                c[ip] += M_ip_i_inv*(lambda[i] - lambda_old[i]);
            }
        }
        alpha = alpha_min + gamma * (alpha - alpha_min);

        real lambda_diff_norm2 = 0.0;
        for (int i = 0; i < num_contact_points; i++) {
            lambda_diff_norm2 += length2(lambda[i] - lambda_old[i]);
        }
        real lambda_norm2 = 0.0;
        for (int i = 0; i < num_contact_points; i++) {
            lambda_norm2 += length2(lambda[i]);
        }
        lambda_err_sq = lambda_diff_norm2 / lambda_norm2;
        if (lambda_err_sq < lambda_err_tol * lambda_err_tol) {
            iter++; break;
        }
    }
    if (iter == max_iters) {
        output_log("Contact solver did not converge! (error = %f)\n", sqrt(lambda_err_sq));
    }
    else {
        output_log("Contact solver converged in %d iters (error = %f)\n", iter, sqrt(lambda_err_sq));
    }

}

void
euler_step_with_collision(ContactSolverType type, uint32_t max_iters,
                          const ArticulatedBodySpec& art, const Material* mat, glm::tvec3<real> gravity, real dt,
                          const real* tau, const tscrew<real>* f_ext, const ContactPoint* contact_points,
                          uint32_t num_contact_points,
                          INOUT real* q, INOUT real* u,
                          OUT real* udot, OUT glm::tvec3<real>* lambda) {
    using namespace Eigen;

    int num_vel_dofs = art.get_num_vel_dofs();
    int num_joints = art.get_num_joints();

    Matrix<real, Dynamic, 1> udot_bar(num_vel_dofs);
    featherstone_forward_dynamics(art, gravity, dt, f_ext, q, u, tau, OUT udot_bar.data());

    if (num_contact_points == 0) {
        integrate_implicit_euler(art, dt, udot_bar.data(), INOUT q, INOUT u);
    }
    else {
        auto t1 = std::chrono::high_resolution_clock::now();

        int num_vel_dofs = art.get_num_vel_dofs();
        int num_joints = art.get_num_joints();

        Matrix<real, Dynamic, 1> u_bar = Map<Matrix<real, Dynamic, 1>>(u, num_vel_dofs) + udot_bar * dt;

        std::vector<ttransform<real>> T_link_global(num_joints), T_joint_global(num_joints);
        calc_transforms(art, q, OUT T_joint_global.data(), OUT T_link_global.data());

        Eigen::Matrix<real, Dynamic, Dynamic> Jc_T(num_vel_dofs, 3*num_contact_points);

        std::vector<tvec3<real>> c(num_contact_points);
        // std::vector<ttransform<real>> art_contact_T(num_contact_points);

        for (int c = 0; c < num_contact_points; c++) {
            const auto& cp = contact_points[c];
            auto [art_id, art_link_idx] = cp.body1_id.get_articulation_id();
            // auto tangent_u = Ez<real>();
            // auto tangent_v = glm::cross(cp.normal, tangent_u);
            // auto contact_T = ttransform<real>(cp.pos, glm::tmat3x3<real>(tangent_u, tangent_v, cp.normal));
            auto contact_T = rtransform(cp.pos, mat3_cast(rotation(Ez<real>(), cp.normal)));
            auto contact_rel_T = contact_T / T_joint_global[art_link_idx];
            // art_contact_T[c] = inverse(contact_rel_T);
            dynmat_view<real> Jc_T_view(Jc_T.data() + 3*c*num_vel_dofs, num_vel_dofs, 3);
            calc_linear_jacobian_transpose(art, art_link_idx, contact_rel_T, T_joint_global.data(),
                                           OUT Jc_T_view);
        }

        Eigen::Matrix<real, Dynamic, 1> tau_star = Jc_T.transpose() * u_bar;

        const real beta = 0.01;
        const real slop = 5e-5;

        for (int i = 0; i < num_contact_points; i++) {
            c[i] = make_vec3<real>(tau_star.data() + 3*i);
            c[i].z -= beta/dt*glm::max<real>(contact_points[i].depth - slop, 0);
        }
        for (int i = 0; i < num_contact_points; i++) {
            lambda[i] = glm::rvec3(0);
        }

        dynmat<tmat3x3<real>> M_contact_inv(num_contact_points, num_contact_points);

        Eigen::Matrix<real, Dynamic, Dynamic> Minv_Jc_T(num_vel_dofs, 3*num_contact_points);
        std::vector<real> zero_vec(num_vel_dofs, 0);

        dynmat_view<real> Minv_Jc_T_view(Minv_Jc_T.data(), num_vel_dofs, 3*num_contact_points);
        dynmat_view<real> Jc_T_view(Jc_T.data(), num_vel_dofs, 3*num_contact_points);

        multiply_inverse_mass_matrix(art, dt, q, Jc_T_view, OUT Minv_Jc_T_view);

        for (int k = 0; k < num_contact_points; k++) {
            Eigen::Matrix<real, Dynamic, 3> Minv_Jck_T = Minv_Jc_T.middleCols<3>(3*k);
            for (int i = 0; i < num_contact_points; i++) {
                Eigen::Matrix<real, 3, 3> M_contact_inv_eigen = Jc_T.middleCols<3>(3*i).transpose() * Minv_Jck_T;
                M_contact_inv(i, k) = glm::make_mat3(M_contact_inv_eigen.data());
            }
        }

        iterative_contact_solver(type, max_iters, mat, dt, num_contact_points, M_contact_inv, INOUT c.data(), INOUT lambda);

        std::vector<rscrew> f_ext_tot(num_joints);
        std::copy_n(f_ext, num_joints, f_ext_tot.data());
        for (int c = 0; c < num_contact_points; c++) {
            const auto& cp = contact_points[c];
            auto [art_id, art_link_idx] = cp.body1_id.get_articulation_id();
            auto contact_T = rtransform(cp.pos, mat3_cast(rotation(Ez<real>(), cp.normal)));
            auto contact_T_rel = T_joint_global[art_link_idx] / contact_T;
            f_ext_tot[art_link_idx] += AdT(contact_T_rel, tscrew<real>(glm::rvec3(0), lambda[c] / dt));
        }

        auto t2 = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1);
        printf("Contact solver: %lld ns\n", duration.count());

        // printf("\n");
        featherstone_forward_dynamics(art, gravity, dt, f_ext_tot.data(), q, u, tau, OUT udot);

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
#if 0
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
#else
        for (int j = 0; j < num_contacts; j++) {
            auto& pt = manifold->getContactPoint(j);
            ContactPoint cp;
            cp.bt_manifold = manifold;
            cp.pos = glmconv(pt.getPositionWorldOnB());
            cp.normal = glmconv(pt.m_normalWorldOnB);
            cp.depth = -pt.getDistance();
            cp.area = 0;
            cp.body1_id = body1_id;
            cp.body2_id = body2_id;
            contact_points.push_back(cp);
        }
#endif
    }
    return contact_points;
}

std::vector<ContactPoint>
contact_points_between_art_links_and_ground(const ArticulatedBodySpec& art, const Id<ArticulatedBody> art_id,
                                            const uint32_t* link_indices, uint32_t link_indices_count,
                                            const ttransform<real>* link_global_trans) {

    std::vector<ContactPoint> contact_points;

    const float epsilon = 1e-7f;
    for (uint32_t li = 0; li < link_indices_count; li++) {
        std::vector<glm::tvec3<real>> cpos;
        uint32_t i = link_indices[li];
        switch (art.links[i].col_shape.type) {
            case artsim::CollisionShape::Type::Box: {
                glm::tvec3<real> ext = real(0.5) * art.links[i].col_shape.scale;
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
#if 0
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
                    ContactPoint cp;
                    cp.bt_manifold = nullptr;
                    cp.pos = glm::tvec3<real>(pos.x, 0, pos.z);
                    cp.normal = Ey<real>();
                    cp.depth = -pos.y;
                    cp.area = 0;
                    cp.body1_id = BodyId::from_articulation_link(art_id, i);
                    cp.body2_id = BodyId::from_ground();
                    contact_points.push_back(cp);
                }
#endif
            } break;
            case artsim::CollisionShape::Type::Sphere: {
                glm::tvec3<real> p = link_global_trans[i].v;
                real r = art.links[i].col_shape.scale.x;
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
