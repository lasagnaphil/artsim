//
// Created by lasagnaphil on 8/4/21.
//

#include <artsim/types.h>
#include <artsim/world.h>
#include <artsim/art_dynamics.h>
#include <chrono>
#include <Tracy.hpp>

using namespace glm;
using namespace glmx;

namespace artsim {

glm::rvec3 contact_proximal_solver(glm::rvec3 lambda, const glmx::rsmat3x3& Minv, glm::rvec3 c, real mu) {
    const real alpha = 1.0f;
    real r_z = alpha / Minv.zz;
    real r_t = alpha / max(Minv.xx, Minv.yy);
    rvec3 v = c + Minv*lambda;
    real lambda_z = max(real(0), lambda.z - r_z*v.z);
    // TODO: Do real euclidean projection on conic disk
    tvec2<real> lambda_t = tvec2<real>(lambda.x - r_t*v.x, lambda.y - r_t*v.y);
    real lambda_t_len = length(lambda_t);
    if (lambda_t_len > mu*lambda_z) {
        lambda_t = mu*lambda_z*normalize(lambda_t);
    }
    return rvec3(lambda_t.x, lambda_t.y, lambda_z);
}


void World::proximal_solver(const ContactPoint* contact_points, int num_contact_points) {
    auto t1 = std::chrono::high_resolution_clock::now();

    using MatrixXr = Eigen::Matrix<real, Eigen::Dynamic, Eigen::Dynamic>;
    using VectorXr = Eigen::Matrix<real, Eigen::Dynamic, 1>;

    std::unordered_map<BodyLinkId, std::vector<int>> body_contact_points_map;
    std::unordered_map<Id<ArticulatedBody>, std::vector<int>> art_contact_points_map;
    std::unordered_map<Id<RigidBody>, std::vector<int>> rb_contact_points_map;

    articulated_bodies.foreach_id([&](Id<ArticulatedBody> art_id) {
        art_contact_points_map.insert({art_id, {}});
        // body_contact_points_map.insert({BodyLinkId::from_articulation_link(art_id, 0), {}});
    });

    rigid_bodies.foreach_id([&](Id<RigidBody> rb_id) {
        rb_contact_points_map.insert({rb_id, {}});
        // body_contact_points_map.insert({BodyLinkId::from_rigid_body(rb_id), {}});
    });

    auto insert_contact_info = [&](BodyLinkId bid, int cidx) {
        if (bid.is_articulation()) {
            auto [art_id, art_lidx] = bid.get_articulation_id();
            art_contact_points_map[art_id].push_back(cidx);
            // bid = BodyLinkId::from_articulation_link(art_id, 0);
        }
        else {
            auto rb_id = bid.get_rigid_body_id();
            auto rb = rigid_bodies.get(rb_id);
            if (rb->spec.is_static) return; // Don't calculate contacts if rigid body is static!
            rb_contact_points_map[rb_id].push_back(cidx);
        }
        // body_contact_points_map[bid].push_back(cidx);
    };

    {
        ZoneNamedN(InsertContactInfo, "InsertContactInfo", true);

        for (int cidx = 0; cidx < num_contact_points; cidx++) {
            insert_contact_info(contact_points[cidx].body1_id, cidx);
            insert_contact_info(contact_points[cidx].body2_id, cidx);
        }
    }

    std::vector<rvec3> c(num_contact_points, rvec3(0));
    std::vector<rvec3> lambda(num_contact_points, rvec3(0));
    std::vector<Material> mat(num_contact_points);
    dynmat<glm::rmat3> M_delassus(num_contact_points, num_contact_points);
    M_delassus.clear_zero();

    {
        ZoneNamedN(CalcDelassusMatrix, "CalcDelassusMatrix", true)

        int i = 0;
        for (auto& [art1_id, cidx_list] : art_contact_points_map) {
            ZoneNamedN(CalcDelassusMatrixForArt, "CalcDelassusMatrixForArt", true)
            ArticulatedBody& art1 = *articulated_bodies.get(art1_id);
            const ArticulatedBodySpec& art1_spec = art1.get_spec();
            int art1_num_joints = art1.get_num_joints();
            int art1_num_vel_dofs = art1.get_num_vel_dofs();
            int art1_num_contact_points = cidx_list.size();

            rscrew* art1_f_ext = art1.get_external_force_buf();
            real* art1_q = art1.get_pos_buf();
            real* art1_u = art1.get_vel_buf();
            real* art1_tau = art1.get_internal_force_buf();
            real* art1_q_target = art1.get_target_pos_buf();

            VectorXr art1_udot_bar(art1_num_vel_dofs);

            featherstone_forward_dynamics(art1_spec, cfg.gravity, cfg.dt,
                                          art1_f_ext, art1_q, art1_u, art1_tau, art1_q_target, OUT art1_udot_bar.data());

            VectorXr art1_u_bar = Eigen::Map<VectorXr>(art1_u, art1_num_vel_dofs) + art1_udot_bar * cfg.dt;

            MatrixXr Jc_T(art1_num_vel_dofs, 3*art1_num_contact_points);

            for (int k = 0; k < cidx_list.size(); k++) {
                int cidx = cidx_list[k];
                const ContactPoint& cp = contact_points[cidx];
                bool body1_is_art1 = cp.body1_id.is_articulation() && cp.body1_id.get_articulation_id().first == art1_id;
                BodyLinkId body1_id = body1_is_art1 ? cp.body1_id : cp.body2_id;
                BodyLinkId body2_id = body1_is_art1 ? cp.body2_id : cp.body1_id;
                auto [_, art1_lidx] = body1_id.get_articulation_id();
                auto contact_T = rtransform(cp.pos, rmat3(cp.tangent1, cp.tangent2, cp.normal));
                rtransform* T_joint_global = art1.get_global_joint_trans_buf();
                dynmat_view<real> Jc_T_view(Jc_T.data(), art1_num_vel_dofs, 3*art1_num_contact_points);
                calc_linear_jacobian_transpose(art1.get_spec(), art1_lidx, contact_T, T_joint_global,
                                               OUT Jc_T_view.slice(0, art1_num_vel_dofs, 3*k, 3));
                // lambda[cidx].x = cp->bt_manifold_point->m_appliedImpulseLateral1;
                // lambda[cidx].y = cp->bt_manifold_point->m_appliedImpulseLateral2;
                // lambda[cidx].z = cp->bt_manifold_point->m_appliedImpulse;
            }

            VectorXr tau_star = Jc_T.transpose() * art1_u_bar;

            const real beta = 0.01;
            const real slop = 5e-5;

            for (int k = 0; k < cidx_list.size(); k++) {
                int cidx = cidx_list[k];
                const ContactPoint& cp = contact_points[cidx];
                bool body1_is_art1 = cp.body1_id.is_articulation() && cp.body1_id.get_articulation_id().first == art1_id;
                BodyLinkId body1_id = body1_is_art1 ? cp.body1_id : cp.body2_id;
                BodyLinkId body2_id = body1_is_art1 ? cp.body2_id : cp.body1_id;
                glm::rvec3 tau = make_vec3<real>(tau_star.data() + 3*k);
                tau.z += beta / cfg.dt * glm::max<real>(cp.distance + slop, 0);
                if (glm::isnan(tau.x) || glm::isnan(tau.y) || glm::isnan(tau.z)) {
                    printf("NaN error!\n");
                }
                if (body1_is_art1) c[cidx] += tau;
                else c[cidx] -= tau;
                Id<Material> body1_mat, body2_mat;
                body1_mat = art1.get_mat_id();
                if (body2_id.is_articulation()) {
                    auto [art2_id, art2_lidx] = body2_id.get_articulation_id();
                    auto art2 = articulated_bodies.get(art2_id);
                    body2_mat = art2->get_mat_id();
                }
                else {
                    auto rb2 = rigid_bodies.get(body2_id.get_rigid_body_id());
                    body2_mat = rb2->mat_id;
                }
                mat[cidx] = material_db.get_material_pair(body1_mat, body2_mat);
            }

            MatrixXr Minv_Jc_T(art1_num_vel_dofs, 3*art1_num_contact_points);
            std::vector<real> zero_vec(art1_num_vel_dofs, 0);

            dynmat_view<real> Minv_Jc_T_view(Minv_Jc_T.data(), art1_num_vel_dofs, 3*art1_num_contact_points);
            dynmat_view<real> Jc_T_view(Jc_T.data(), art1_num_vel_dofs, 3*art1_num_contact_points);

            multiply_inverse_mass_matrix(art1_spec, cfg.dt, art1_q, Jc_T_view, OUT Minv_Jc_T_view);

            {
                ZoneNamedN(CalcDelassusMatrixMain, "CalcDelassusMatrixMain", true)
                MatrixXr M_delassus_eigen = Jc_T.transpose() * Minv_Jc_T;
                for (int k = 0; k < art1_num_contact_points; k++) {
                    for (int i = 0; i < art1_num_contact_points; i++) {
                        Eigen::Matrix<real, 3, 3> M_contact_inv_eigen = M_delassus_eigen.block<3, 3>(3*i, 3*k);
                        M_delassus(cidx_list[i], cidx_list[k]) += glm::make_mat3(M_contact_inv_eigen.data());
                    }
                }
            }
        }

        for (auto& [rb1_id, cidx_list] : rb_contact_points_map) {
            // TODO
        }
    }

    {
        ZoneNamedN(SolveContacts, "SolveContacts", true);
        std::vector<tvec3<real>> lambda_old(num_contact_points);
        bool converged = false;
        const real lambda_err_tol = 1e-4;
        real lambda_err_sq;
        int iter;
        for (iter = 0; iter < cfg.max_iters; iter++) {
            std::copy(lambda.begin(), lambda.end(), lambda_old.begin());

            for (int i = 0; i < num_contact_points; i++) {
                // Find solution for one contact force
                if (c[i].z > 0) {
                    lambda[i] = glm::vec3(0);
                }
                else {
                    real mu = mat[i].friction;
                    rsmat3x3 M_inv_ii = glmx::smat3_cast(M_delassus(i, i));
                    rvec3 lambda_v0 = -inverse(M_inv_ii) * c[i];
                    if (mu*mu * lambda_v0.z*lambda_v0.z >= lambda_v0.x*lambda_v0.x + lambda_v0.y*lambda_v0.y) {
                        lambda[i] = lambda_v0;
                    }
                    else {
                        tvec3<real> lambda_star = contact_proximal_solver(lambda[i], M_inv_ii, c[i], mu);
                        if (glm::isnan(lambda_star[0]) || glm::isnan(lambda_star[1]) || glm::isnan(lambda_star[2])) {
                            output_log("NaN error!\n");
                        }
                        lambda[i] = lambda_star;
                    }
                }
                // Update velocities via sequential impulse
                /*
                for (int ip = 0; ip < num_contact_points; ip++) {
                    if (i == ip) continue;
                    c[ip] += M_delassus(ip, i)*(lambda[i] - lambda_old[i]);
                }
                 */
                auto& cp = contact_points[i];
                std::vector<int> *cidx_list1, *cidx_list2;
                if (cp.body1_id.is_articulation()) {
                    cidx_list1 = &art_contact_points_map[cp.body1_id.get_articulation_id().first];
                }
                else {
                    cidx_list1 = &rb_contact_points_map[cp.body1_id.get_rigid_body_id()];
                }
                if (cp.body2_id.is_articulation()) {
                    cidx_list2 = &art_contact_points_map[cp.body2_id.get_articulation_id().first];
                }
                else {
                    cidx_list2 = &rb_contact_points_map[cp.body2_id.get_rigid_body_id()];
                }
                for (int cidx : *cidx_list1) {
                    if (cidx == i) continue;
                    c[cidx] += M_delassus(cidx, i)*(lambda[i] - lambda_old[i]);
                }
                for (int cidx : *cidx_list2) {
                    if (cidx == i) continue;
                    c[cidx] -= M_delassus(cidx, i)*(lambda[i] - lambda_old[i]);
                }
            }

            real lambda_diff_norm2 = 0.0;
            for (int i = 0; i < num_contact_points; i++) {
                lambda_diff_norm2 += length2(lambda[i] - lambda_old[i]);
            }
            real lambda_norm2 = 0.0;
            for (int i = 0; i < num_contact_points; i++) {
                lambda_norm2 += length2(lambda[i]);
            }
            if (lambda_norm2 < 1e-12) {
                lambda_err_sq = 0;
            }
            else {
                lambda_err_sq = lambda_diff_norm2 / lambda_norm2;
            }
            if (lambda_err_sq < lambda_err_tol * lambda_err_tol) {
                converged = true;
                iter++; break;
            }
        }

        real lambda_err = sqrt(lambda_err_sq);
        if (converged) {
            output_log("Contact solver converged in %d iters (error = %f)\n", iter, lambda_err);
        }
        else {
            output_log("Contact solver did not converge! (error = %f)\n", lambda_err);
        }
    }

    auto t2 = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1);
    output_log("Contact solver: %lld ns\n", duration.count());

    {
        ZoneNamedN(IntegrateWithContacts, "IntegrateWithContacts", true);

        articulated_bodies.foreach_id_val([&](Id<ArticulatedBody> art_id, ArticulatedBody& art) {
            int num_joints = art.get_num_joints();
            std::vector<rscrew> f_ext_tot(num_joints);
            std::copy_n(art.get_external_force_buf(), num_joints, f_ext_tot.data());
            auto it = art_contact_points_map.find(art_id);
            if (it != art_contact_points_map.end()) {
                auto& cidx_list = it->second;
                for (int cidx : cidx_list) {
                    auto& cp = contact_points[cidx];
                    bool body1_is_art1 = cp.body1_id.is_articulation() &&
                            cp.body1_id.get_articulation_id().first == art_id;
                    BodyLinkId body1_id = body1_is_art1 ? cp.body1_id : cp.body2_id;
                    BodyLinkId body2_id = body1_is_art1 ? cp.body2_id : cp.body1_id;
                    auto [_, art_lidx] = body1_id.get_articulation_id();
                    auto contact_T = rtransform(cp.pos, mat3_cast(rotation(Ez<real>(), cp.normal)));
                    auto contact_rel_T = art.get_global_joint_trans(art_lidx) / contact_T;
                    f_ext_tot[art_lidx] += AdT(contact_rel_T, rscrew(rvec3(0), lambda[cidx] / cfg.dt));
                    // cp->bt_manifold_point->m_appliedImpulseLateral1 = lambda[cidx].x;
                    // cp->bt_manifold_point->m_appliedImpulseLateral2 = lambda[cidx].y;
                    // cp->bt_manifold_point->m_appliedImpulse = lambda[cidx].z;
                }
            }

            auto& spec = art.get_spec();
            real* q = art.get_pos_buf(); real* u = art.get_vel_buf(); real* udot = art.get_acc_buf();
            featherstone_forward_dynamics(spec, cfg.gravity, cfg.dt,
                                          f_ext_tot.data(), q, u,
                                          art.get_internal_force_buf(), art.get_target_pos_buf(), OUT udot);
            integrate_implicit_euler(spec, cfg.dt, udot, INOUT q, INOUT u);
        });

        rigid_bodies.foreach_id_val([&](Id<RigidBody> rb_id, RigidBody& rb) {
            // TODO
        });
    }
}

}
