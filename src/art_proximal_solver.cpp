//
// Created by lasagnaphil on 8/4/21.
//

/*
 * Implementation of "Rigid Body Contact Problems using Proximal Operators" by Kenny Erleben.
 * TODO:
 * - Implement midpoint integration
 * - In positional updates, the displacement vector g should be updated after a single iteration.
 *     (This is the reason why pos_iter > 1 doesn't work right now)
 * - Need to tune initial R values, both for velocity and position update.
 * - Need to find out why simulation explodes with human articulation
 */

#include <artsim/types.h>
#include <artsim/math/eigen.h>
#include <artsim/world.h>
#include <artsim/art_dynamics.h>
#include <chrono>
#include <Tracy.hpp>

using namespace glm;
using namespace glmx;

namespace artsim {

// #define PROXIMAL_SOLVER_JACOBI
#define PROXIMAL_SOLVER_GAUSS_SEIDEL
// #define PROXIMAL_SOLVER_GLOBAL_R_STRATEGY
#define PROXIMAL_SOLVER_LOCAL_R_STRATEGY

void World::proximal_solver() {
    auto t1 = std::chrono::high_resolution_clock::now();

    using MatrixXr = Eigen::Matrix<real, Eigen::Dynamic, Eigen::Dynamic>;
    using VectorXr = Eigen::Matrix<real, Eigen::Dynamic, 1>;

    int num_contact_points = contact_points.size();

    std::unordered_map<BodyId, std::vector<std::pair<int, int>>> contact_info;
    for (int cid = 0; cid < num_contact_points; cid++) {
        auto& c = contact_points[cid];
        BodyId bid1 = c.body1_id.get_body_id();
        auto it1 = contact_info.find(bid1);
        if (it1 == contact_info.end()) {
            contact_info.insert({bid1, {{cid, 1}}});
        }
        else {
            contact_info.at(bid1).push_back({cid, 1});
        }
        BodyId bid2 = c.body2_id.get_body_id();
        auto it2 = contact_info.find(bid2);
        if (it2 == contact_info.end()) {
            contact_info.insert({bid2, {{cid, -1}}});
        }
        else {
            contact_info.at(bid2).push_back({cid, -1});
        }
    }

    std::unordered_map<BodyId, int> body_id_to_entity_id;
    std::vector<BodyId> entity_id_to_body_id;
    std::vector<std::vector<std::pair<int, int>>> entity_id_to_contact_ids;
    std::vector<std::pair<int, int>> entity_id_to_range;
    std::vector<std::pair<int, int>> contact_id_to_rel_id(num_contact_points, {-1, -1});

    int count = 0;
    int vel_idx = 0;
    for (auto& [bid, contact_list] : contact_info) {
        int num_vel_dof;
        if (bid.is_articulation()) {
            auto art_id = bid.get_art_id();
            auto& art = *get_articulated_body(art_id);
            num_vel_dof = art.get_num_vel_dofs();
        }
        else {
            auto rb_id = bid.get_rigid_body_id();
            auto& rb = *get_rigid_body(rb_id);
            if (rb.spec.is_static) continue;
            num_vel_dof = 6;
        }

        body_id_to_entity_id.insert({bid, count});
        entity_id_to_body_id.push_back(bid);
        entity_id_to_contact_ids.push_back(contact_list);
        entity_id_to_range.push_back({vel_idx, num_vel_dof});
        for (int i = 0; i < contact_list.size(); i++) {
            auto [cid, sign] = contact_list[i];
            if (sign == 1) contact_id_to_rel_id[cid].first = i;
            else contact_id_to_rel_id[cid].second = i;
        }

        count++;
        vel_idx += num_vel_dof;
    }

    const int num_entities = count;
    const int num_total_vel_dofs = vel_idx;

    VectorXr lam(3*num_contact_points);
    VectorXr g(num_contact_points);
    std::vector<Material> materials(num_contact_points);
    std::vector<MatrixXr> Jt_list(num_entities);
    std::vector<MatrixXr> Minv_Jt_list(num_entities);
#ifdef PROXIMAL_SOLVER_LOCAL_R_STRATEGY
    std::vector<MatrixXr> J_Minv_Jt_list(num_entities);
#endif
    std::vector<VectorXr> b_list(num_entities);

    {
        ZoneNamedN(PrecalculateMatrices, "PrecalculateMatrices", true)

        for (int cid = 0; cid < num_contact_points; cid++) {
            auto& contact = contact_points[cid];
            Id<Material> mat1_id = get_material(contact.body1_id);
            Id<Material> mat2_id = get_material(contact.body2_id);
            materials[cid] = material_db.get_material_pair(mat1_id, mat2_id);
            g(cid) = contact.distance;
        }

        for (int eid = 0; eid < num_entities; eid++) {
            BodyId bid = entity_id_to_body_id[eid];
            auto& contact_list = entity_id_to_contact_ids[eid];
            auto& Jt = Jt_list[eid];
            auto& Minv_Jt = Minv_Jt_list[eid];
#ifdef PROXIMAL_SOLVER_LOCAL_R_STRATEGY
            auto& J_Minv_Jt = J_Minv_Jt_list[eid];
#endif
            auto& b = b_list[eid];

            if (bid.is_articulation()) {
                auto art_id = bid.get_art_id();
                ArticulatedBody& art = *articulated_bodies.get(art_id);

                const ArticulatedBodySpec& art_spec = art.get_spec();
                int art_num_joints = art.get_num_joints();
                int art_num_vel_dofs = art.get_num_vel_dofs();
                rscrew* art_f_ext = art.get_external_force_buf();
                real* art_q = art.get_pos_buf();
                real* art_u = art.get_vel_buf();
                real* art_tau = art.get_internal_force_buf();
                real* art_q_target = art.get_target_pos_buf();

                Jt.resize(art_num_vel_dofs, 3*contact_list.size());
                Minv_Jt.resize(art_num_vel_dofs, 3*contact_list.size());
#ifdef PROXIMAL_SOLVER_LOCAL_R_STRATEGY
                J_Minv_Jt.resize(3*contact_list.size(), 3*contact_list.size());
#endif
                b.resize(3*contact_list.size());
                b.setZero();

                for (int k = 0; k < contact_list.size(); k++) {
                    auto [cid, sign] = contact_list[k];
                    const ContactPoint& cp = contact_points[cid];
                    BodyLinkId blid = sign? cp.body1_id : cp.body2_id;
                    auto [_, art_lidx] = blid.get_articulation_id();
                    auto contact_T = rtransform(cp.pos, rmat3(cp.tangent1, cp.tangent2, cp.normal));
                    rtransform* T_joint_global = art.get_global_joint_trans_buf();
                    dynmat_view<real> Jt_view(Jt.data(), art_num_vel_dofs, 3*contact_list.size());
                    calc_linear_jacobian_transpose(art.get_spec(), art_lidx, contact_T, T_joint_global,
                                                   OUT Jt_view.slice(0, art_num_vel_dofs, 3*k, 3));
                }

                multiply_inverse_mass_matrix(art_spec, cfg.dt, art_q, get_view(Jt), OUT get_view(Minv_Jt));
#ifdef PROXIMAL_SOLVER_LOCAL_R_STRATEGY
                J_Minv_Jt = Jt.transpose() * Minv_Jt;
#endif

                VectorXr art_udot_bar(art_num_vel_dofs);
                featherstone_forward_dynamics(art_spec, cfg.gravity, cfg.dt,
                                              art_f_ext, art_q, art_u, art_tau, art_q_target, OUT art_udot_bar.data());
                VectorXr J_u = Jt.transpose() * Eigen::Map<VectorXr>(art_u, art_num_vel_dofs);
                VectorXr J_du = Jt.transpose() * (cfg.dt * art_udot_bar);
                for (int k = 0; k < contact_list.size(); k++) {
                    auto [cid, sign] = contact_list[k];
                    auto& mat = materials[cid];
                    Vector3r E(real(1), real(1), real(1) + mat.restitution);
                    b.middleRows<3>(3*k) += (real)sign * E.cwiseProduct(J_u.middleRows<3>(3*k)) + J_du.middleRows<3>(3*k);
                }
            }
            else {
                // TODO
            }
        }
    }

    {
        ZoneNamedN(VelocityUpdate, "VelocityUpdate", true);

        VectorXr w(num_total_vel_dofs);
        VectorXr z(3*num_contact_points);

        VectorXr lam_old(3*num_contact_points);
        lam.setZero();
        int iter;

        VectorXr R(3*num_contact_points);
#if defined(PROXIMAL_SOLVER_GLOBAL_R_STRATEGY)
        R.fill(1.0);
#elif defined(PROXIMAL_SOLVER_LOCAL_R_STRATEGY)
        for (int cid = 0; cid < num_contact_points; cid++) {
            auto& cp = contact_points[cid];
            int crelid1 = contact_id_to_rel_id[cid].first;
            int crelid2 = contact_id_to_rel_id[cid].second;
            int eid1 = body_id_to_entity_id[cp.body1_id.get_body_id()];
            int eid2 = body_id_to_entity_id[cp.body2_id.get_body_id()];
            auto& J_Minv_Jt1 = J_Minv_Jt_list[eid1];
            auto& J_Minv_Jt2 = J_Minv_Jt_list[eid2];
            Eigen::Matrix<real, 3, 3> A;
            A.setZero();
            if (crelid1 != -1)
                A += J_Minv_Jt1.block<3, 3>(3*crelid1, 3*crelid1);
            if (crelid2 != -1)
                A += J_Minv_Jt2.block<3, 3>(3*crelid2, 3*crelid2);
            R.middleRows<3>(3*cid) = A.diagonal().cwiseInverse();
        }
        // std::cout << "R: " << R.transpose() << std::endl;
#endif
        real r = std::numeric_limits<real>::max(), r_old;

#if defined(PROXIMAL_SOLVER_JACOBI)
        for (iter = 0; iter < cfg.max_vel_iters; iter++) {
            lam_old = lam;
            r_old = r;
            // Update w, z
            w.setZero();
            for (int cid = 0; cid < num_contact_points; cid++) {
                auto& cp = contact_points[cid];
                auto& mat = materials[cid];
                int crelid1 = contact_id_to_rel_id[cid].first;
                int crelid2 = contact_id_to_rel_id[cid].second;
                int eid1 = body_id_to_entity_id[cp.body1_id.get_body_id()];
                int eid2 = body_id_to_entity_id[cp.body2_id.get_body_id()];
                auto& Minv_Jt1 = Minv_Jt_list[eid1];
                auto& Minv_Jt2 = Minv_Jt_list[eid2];
                auto [dof_start1, dof_size1] = entity_id_to_range[eid1];
                auto [dof_start2, dof_size2] = entity_id_to_range[eid2];
                if (crelid1 != -1)
                    w.middleRows(dof_start1, dof_size1) += Minv_Jt1.middleCols<3>(3*crelid1) * lam.middleRows<3>(3*cid);
                if (crelid2 != -1)
                    w.middleRows(dof_start2, dof_size2) -= Minv_Jt2.middleCols<3>(3*crelid2) * lam.middleRows<3>(3*cid);
            }
            z = lam;
            for (int cid = 0; cid < num_contact_points; cid++) {
                auto& cp = contact_points[cid];
                auto& mat = materials[cid];
                int crelid1 = contact_id_to_rel_id[cid].first;
                int crelid2 = contact_id_to_rel_id[cid].second;
                int eid1 = body_id_to_entity_id[cp.body1_id.get_body_id()];
                int eid2 = body_id_to_entity_id[cp.body2_id.get_body_id()];
                auto& Jt1 = Jt_list[eid1];
                auto& Jt2 = Jt_list[eid2];
                auto& b1 = b_list[eid1];
                auto& b2 = b_list[eid2];
                auto [dof_start1, dof_size1] = entity_id_to_range[eid1];
                auto [dof_start2, dof_size2] = entity_id_to_range[eid2];
                if (crelid1 != -1)
                    z.middleRows<3>(3*cid) -= R.middleRows<3>(3*cid).cwiseProduct(
                            Jt1.middleCols<3>(3*crelid1).transpose() * w.middleRows(dof_start1, dof_size1) + b1.middleRows<3>(3*cid));
                if (crelid2 != -1)
                    z.middleRows<3>(3*cid) += R.middleRows<3>(3*cid).cwiseProduct(
                            Jt2.middleCols<3>(3*crelid2).transpose() * w.middleRows(dof_start2, dof_size2) + b2.middleRows<3>(3*cid));
            }
            // std::cout << "w: " << w.transpose() << std::endl;
            // std::cout << "z: " << z.transpose() << std::endl;
            // Update for all contacts
            for (int cid = 0; cid < num_contact_points; cid++) {
                auto& mat = materials[cid];
                glm::rvec2 z_t = {z(3*cid), z(3*cid+1)};
                real z_n = z(3*cid+2);
                real lam_n = glm::max(real(0), z_n);
                real z_t_len = glm::length(z_t);
                glm::rvec2 lam_t = z_t;
                if (z_t_len > mat.friction * lam_n) {
                    lam_t = (mat.friction * lam_n / z_t_len) * lam_t;
                }
                lam(3*cid+0) = lam_t.x;
                lam(3*cid+1) = lam_t.y;
                lam(3*cid+2) = lam_n;
            }
            r = (lam - lam_old).lpNorm<Eigen::Infinity>();
            printf("velocity error: %f\n", r);
            // Adaptive r-factor tuning
            if (r > r_old) {
                R *= 0.5;
                lam = lam_old;
                iter--;
            }
        }
#endif

#if defined(PROXIMAL_SOLVER_GAUSS_SEIDEL)
        for (iter = 0; iter < cfg.max_vel_iters; iter++) {
            lam_old = lam;
            r_old = r;
            w.setZero();
            for (int cid = 0; cid < num_contact_points; cid++) {
                auto& cp = contact_points[cid];
                auto& mat = materials[cid];
                int crelid1 = contact_id_to_rel_id[cid].first;
                int crelid2 = contact_id_to_rel_id[cid].second;
                int eid1 = body_id_to_entity_id[cp.body1_id.get_body_id()];
                int eid2 = body_id_to_entity_id[cp.body2_id.get_body_id()];
                auto& Minv_Jt1 = Minv_Jt_list[eid1];
                auto& Minv_Jt2 = Minv_Jt_list[eid2];
                auto [dof_start1, dof_size1] = entity_id_to_range[eid1];
                auto [dof_start2, dof_size2] = entity_id_to_range[eid2];
                if (crelid1 != -1)
                    w.middleRows(dof_start1, dof_size1) += Minv_Jt1.middleCols<3>(3*crelid1) * lam.middleRows<3>(3*cid);
                if (crelid2 != -1)
                    w.middleRows(dof_start2, dof_size2) -= Minv_Jt2.middleCols<3>(3*crelid2) * lam.middleRows<3>(3*cid);
            }
            // std::cout << "w: " << w.transpose() << std::endl;
            z.setZero();
            for (int cid = 0; cid < num_contact_points; cid++) {
                auto& cp = contact_points[cid];
                auto& mat = materials[cid];
                int crelid1 = contact_id_to_rel_id[cid].first;
                int crelid2 = contact_id_to_rel_id[cid].second;
                int eid1 = body_id_to_entity_id[cp.body1_id.get_body_id()];
                int eid2 = body_id_to_entity_id[cp.body2_id.get_body_id()];
                auto& Jt1 = Jt_list[eid1];
                auto& Jt2 = Jt_list[eid2];
                auto& Minv_Jt1 = Minv_Jt_list[eid1];
                auto& Minv_Jt2 = Minv_Jt_list[eid2];
                auto& b1 = b_list[eid1];
                auto& b2 = b_list[eid2];
                auto [dof_start1, dof_size1] = entity_id_to_range[eid1];
                auto [dof_start2, dof_size2] = entity_id_to_range[eid2];
                Vector3r z_c = lam.middleRows<3>(3*cid);
                if (crelid1 != -1)
                    z_c -= R.middleRows<3>(3*cid).cwiseProduct(
                            Jt1.middleCols<3>(3*crelid1).transpose() * w.middleRows(dof_start1, dof_size1) + b1.middleRows<3>(3*crelid1));
                if (crelid2 != -1)
                    z_c += R.middleRows<3>(3*cid).cwiseProduct(
                            Jt2.middleCols<3>(3*crelid2).transpose() * w.middleRows(dof_start2, dof_size2) + b2.middleRows<3>(3*crelid2));
                z.middleRows<3>(3*cid) = z_c;

                glm::rvec2 z_t = {z_c(0), z_c(1)};
                real z_n = z_c(2);
                real lam_n = glm::max(real(0), z_n);
                real z_t_len = glm::length(z_t);
                glm::rvec2 lam_t = z_t;
                if (z_t_len > mat.friction * lam_n) {
                    lam_t = (mat.friction * lam_n / z_t_len) * lam_t;
                }
                Vector3r lam_prime(lam_t.x, lam_t.y, lam_n);
                Vector3r dlam = lam_prime - lam.middleRows<3>(3*cid);
                if (crelid1 != -1)
                    w.middleRows(dof_start1, dof_size1) += Minv_Jt1.middleCols<3>(3*crelid1) * dlam;
                if (crelid2 != -1)
                    w.middleRows(dof_start2, dof_size2) -= Minv_Jt2.middleCols<3>(3*crelid2) * dlam;
                lam.middleRows<3>(3*cid) = lam_prime;
            }
            // std::cout << "z: " << z.transpose() << std::endl;
            // std::cout << "lam: " << lam.transpose() << std::endl;

            r = (lam - lam_old).lpNorm<Eigen::Infinity>();
            printf("velocity error: %f\n", r);
            // Adaptive r-factor tuning
            if (r > r_old) {
#if defined(PROXIMAL_SOLVER_GLOBAL_R_STRATEGY)
                R *= 0.5;
#elif defined(PROXIMAL_SOLVER_LOCAL_R_STRATEGY)
                R *= 0.9;
#endif
                lam = lam_old;
                iter--;
            }
        }
#endif

        printf("Contact solver velocity error: %f\n", r);
    }

    for (int cid = 0; cid < num_contact_points; cid++) {
        auto& cp = contact_points[cid];
        cp.lam = eigen_to_glm(lam.middleRows<3>(3*cid));
    }

    {
        ZoneNamedN(IntegrateVelocity, "IntegrateVelocity", true);

        articulated_bodies.foreach_id_val([&](Id<ArticulatedBody> art_id, ArticulatedBody& art) {
            BodyId bid = BodyId::from_articulated_body(art_id);
            auto it = body_id_to_entity_id.find(bid);
            if (it == body_id_to_entity_id.end()) {
                art.forward_dynamics(cfg.gravity, cfg.dt);
                art.integrate(cfg.dt);
            }
            else {
                int eid = it->second;
                auto& contact_list = entity_id_to_contact_ids[eid];
                int num_joints = art.get_num_joints();
                std::vector<rscrew> f_ext_tot(num_joints);
                std::copy_n(art.get_external_force_buf(), num_joints, f_ext_tot.data());
                for (auto [cid, sign] : contact_list) {
                    auto& cp = contact_points[cid];
                    BodyLinkId blid = sign == 1? cp.body1_id : cp.body2_id;
                    auto [_, art_lidx] = blid.get_articulation_id();
                    auto contact_frame = rtransform(cp.pos, mat3(cp.tangent1, cp.tangent2, cp.normal));
                    auto contact_rel_frame = art.get_global_joint_trans(art_lidx) / contact_frame;
                    glm::rvec3 lam_i = (real)sign * eigen_to_glm(lam.middleRows<3>(3*cid));
                    f_ext_tot[art_lidx] += AdT(contact_rel_frame, rscrew(rvec3(0), lam_i / cfg.dt));
                    // cp->bt_manifold_point->m_appliedImpulseLateral1 = lambda[cidx].x;
                    // cp->bt_manifold_point->m_appliedImpulseLateral2 = lambda[cidx].y;
                    // cp->bt_manifold_point->m_appliedImpulse = lambda[cidx].z;
                }

                auto& spec = art.get_spec();
                featherstone_forward_dynamics(spec, cfg.gravity, cfg.dt,
                                              f_ext_tot.data(), art.get_pos_buf(), art.get_vel_buf(),
                                              art.get_internal_force_buf(), art.get_target_pos_buf(),
                                              OUT art.get_acc_buf());
                art.integrate(cfg.dt);
            }
        });

        rigid_bodies.foreach_id_val([&](Id<RigidBody> rb_id, RigidBody& rb) {
            // TODO
        });
    }

    {
        ZoneNamedN(PositionUpdate, "PositionUpdate", true);

        real R = 0.1; // Global r-Factor strategy
        VectorXr w(num_total_vel_dofs);
        VectorXr g_prime(num_contact_points);
        VectorXr lam_n(num_contact_points);
        VectorXr lam_n_old(3*num_contact_points);
        real r = std::numeric_limits<real>::max(), r_old;

        lam_n.setZero();

        int iter;
        for (iter = 0; iter < cfg.max_pos_iters; iter++) {
            if (iter != 0) {
                for (int cid = 0; cid < num_contact_points; cid++) {
                    auto& cp = contact_points[cid];
                    glm::rvec3 pos1, pos2;
                    if (cp.body1_id.is_articulation()) {
                        auto [art_id, lidx] = cp.body1_id.get_articulation_id();
                        auto art = get_articulated_body(art_id);
                        pos1 = (art->get_global_joint_trans(lidx) * cp.body1_rel_trans).v;
                    }
                    else {
                        auto rb = get_rigid_body(cp.body1_id.get_rigid_body_id());
                        pos1 = (glmx::rtransform(rb->pos, glm::mat3_cast(rb->rot)) * cp.body1_rel_trans).v;
                    }
                    if (cp.body2_id.is_articulation()) {
                        auto [art_id, lidx] = cp.body2_id.get_articulation_id();
                        auto art = get_articulated_body(art_id);
                        pos2 = (art->get_global_joint_trans(lidx) * cp.body2_rel_trans).v;
                    }
                    else {
                        auto rb = get_rigid_body(cp.body2_id.get_rigid_body_id());
                        pos2 = (glmx::rtransform(rb->pos, glm::mat3_cast(rb->rot)) * cp.body2_rel_trans).v;
                    }
                    g(cid) = glm::dot(cp.normal, pos1 - pos2);
                }
            }

            lam_n_old = lam_n;
            r_old = r;
            g_prime = g;
            w.setZero();

            for (int eid = 0; eid < num_entities; eid++) {
                auto [dof_start, dof_size] = entity_id_to_range[eid];
                auto& contact_list = entity_id_to_contact_ids[eid];
                auto& Jt = Jt_list[eid];
                auto& Minv_Jt = Minv_Jt_list[eid];

                for (int k = 0; k < contact_list.size(); k++) {
                    auto [cid, sign] = contact_list[k];
                    real lam_c = (real)sign * lam_n(cid);
                    w.middleRows(dof_start, dof_size) += Minv_Jt.col(3*k+2) * lam_c;
                }
                for (int k = 0; k < contact_list.size(); k++) {
                    auto [cid, sign] = contact_list[k];
                    g_prime(cid) += Jt.col(3*k+2).transpose() * w.middleRows(dof_start, dof_size);
                }
            }
            lam_n = (lam_n - R*g_prime).cwiseMax(0);
            r = (lam_n - lam_n_old).lpNorm<Eigen::Infinity>();
            if (r > r_old) {
                R *= 0.5;
                lam_n = lam_n_old;
                iter--;
            }

            // output_log("Contact solver position error: %f\n", r);
            VectorXr u(num_total_vel_dofs);
            u.setZero();
            for (int eid = 0; eid < num_entities; eid++) {
                BodyId bid = entity_id_to_body_id[eid];
                auto [dof_start, dof_size] = entity_id_to_range[eid];
                auto& contact_list = entity_id_to_contact_ids[eid];
                auto& Minv_Jt = Minv_Jt_list[eid];

                for (int k = 0; k < contact_list.size(); k++) {
                    auto [cid, sign] = contact_list[k];
                    real lam_c = (real)sign * lam_n(cid);
                    u.middleRows(dof_start, dof_size) += Minv_Jt.col(3*k+2) * (lam_c / cfg.dt);
                }
                if (bid.is_articulation()) {
                    auto art_id = bid.get_art_id();
                    auto& art = *get_articulated_body(art_id);
                    integrate_implicit_euler(art.get_spec(), cfg.dt, nullptr,
                                             INOUT art.get_pos_buf(), INOUT u.data() + dof_start);
                    art.forward_kinematics();
                }
                else {
                    // TODO
                }
            }
        }
    }

    auto t2 = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1);
    output_log("Contact solver: %lld ns\n", duration.count());

}

Id<Material> World::get_material(BodyLinkId blid) {
    if (blid.is_articulation()) {
        auto [art_id, lidx] = blid.get_articulation_id();
        auto& art = *get_articulated_body(art_id);
        auto mat_id = art.get_mat_id();
        auto& link = art.get_spec().links[lidx];
        auto mat_link_id = link.mat_id;
        if (mat_id.is_null()) {
            if (mat_link_id.is_null()) {
                printf("Empty material for articulated link {}!", art_id.to_int64());
            }
            return mat_link_id;
        }
        else return mat_id;
    }
    else {
        auto rb_id = blid.get_rigid_body_id();
        auto& rb = *get_rigid_body(rb_id);
        auto mat_id = rb.mat_id;
        if (mat_id.is_null()) {
            printf("Empty material for rigid body {}!", rb_id.to_int64());
        }
        return mat_id;
    }
};


}
