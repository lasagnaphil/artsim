//
// Created by lasagnaphil on 8/4/21.
//

#include <artsim/types.h>
#include <artsim/math/eigen.h>
#include <artsim/world.h>
#include <artsim/art_dynamics.h>
#include <chrono>
#include <Tracy.hpp>

using namespace glm;
using namespace glmx;

namespace artsim {

void World::proximal_solver(const ContactPoint* contact_points, int num_contact_points) {
    auto t1 = std::chrono::high_resolution_clock::now();

    using MatrixXr = Eigen::Matrix<real, Eigen::Dynamic, Eigen::Dynamic>;
    using VectorXr = Eigen::Matrix<real, Eigen::Dynamic, 1>;

    std::unordered_map<BodyId, std::vector<std::pair<int, bool>>> contact_info;
    for (int cid = 0; cid < num_contact_points; cid++) {
        auto& c = contact_points[cid];
        BodyId bid1 = c.body1_id.get_body_id();
        auto it1 = contact_info.find(bid1);
        if (it1 == contact_info.end()) {
            contact_info.insert({bid1, {{cid, true}}});
        }
        else {
            contact_info.at(bid1).push_back({cid, true});
        }
        BodyId bid2 = c.body2_id.get_body_id();
        auto it2 = contact_info.find(bid2);
        if (it2 == contact_info.end()) {
            contact_info.insert({bid2, {{cid, false}}});
        }
        else {
            contact_info.at(bid2).push_back({cid, false});
        }
    }

    std::unordered_map<BodyId, int> body_id_to_entity_id;
    std::vector<BodyId> entity_id_to_body_id;
    std::vector<std::vector<std::pair<int, bool>>> entity_id_to_contact_ids;
    std::vector<std::pair<int, int>> entity_id_to_range;

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
    std::vector<VectorXr> b_list(num_entities);

    {
        ZoneNamedN(PrecalculateMatrices, "PrecalculateMatrices", true)

        auto get_material = [&](BodyLinkId blid) -> Id<Material> {
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

                VectorXr art_udot_bar(art_num_vel_dofs);
                featherstone_forward_dynamics(art_spec, cfg.gravity, cfg.dt,
                                              art_f_ext, art_q, art_u, art_tau, art_q_target, OUT art_udot_bar.data());
                VectorXr J_u = Jt.transpose() * Eigen::Map<VectorXr>(art_u, art_num_vel_dofs);
                VectorXr J_du = Jt.transpose() * (cfg.dt * art_udot_bar);
                for (int k = 0; k < contact_list.size(); k++) {
                    auto [cid, sign] = contact_list[k];
                    auto& mat = materials[cid];
                    Vector3r b_k = (real(1) + mat.restitution) * J_u.middleRows<3>(3*k) + J_du.middleRows<3>(3*k);
                    if (sign) {
                        b.middleRows<3>(3*k) += b_k;
                    }
                    else {
                        b.middleRows<3>(3*k) -= b_k;
                    }
                }
            }
            else {
                // TODO
            }
        }
    }

    {
        ZoneNamedN(VelocityUpdate, "VelocityUpdate", true);

        real R = 5; // Global r-Factor strategy
        VectorXr w(num_total_vel_dofs);
        VectorXr z(3*num_contact_points);
        real r = std::numeric_limits<real>::max(), r_old;

#define PROXIMAL_SOLVER_JACOBI
        VectorXr lam_old(3*num_contact_points);
        lam.setZero();
        int iter;
        for (iter = 0; iter < cfg.max_vel_iters; iter++) {
            lam_old = lam;
            r_old = r;
#ifdef PROXIMAL_SOLVER_JACOBI
            // Update w, z
            z.setZero();
            for (int eid = 0; eid < num_entities; eid++) {
                auto [dof_start, dof_size] = entity_id_to_range[eid];
                auto& contact_list= entity_id_to_contact_ids[eid];
                auto& Jt = Jt_list[eid];
                auto& Minv_Jt = Minv_Jt_list[eid];
                auto& b = b_list[eid];

                VectorXr lam_c(3*contact_list.size());
                for (int k = 0; k < contact_list.size(); k++) {
                    auto [cid, sign] = contact_list[k];
                    lam_c.middleRows<3>(3*k) = lam.middleRows<3>(3*cid);
                }
                w.middleRows(dof_start, dof_size) = Minv_Jt * lam_c;
                VectorXr z_c = lam_c - R * (Jt.transpose() * w.middleRows(dof_start, dof_size) + b);
                for (int k = 0; k < contact_list.size(); k++) {
                    auto [cid, sign] = contact_list[k];
                    z.middleRows<3>(3*cid) += z_c.middleRows<3>(3*k);
                }
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
            // Adaptive r-factor tuning
            if (r > r_old) {
                R *= 0.5;
                lam = lam_old;
                iter--;
            }
#endif

        }

        // output_log("Contact solver velocity error: %f\n", r);
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
                    BodyLinkId blid = sign? cp.body1_id : cp.body2_id;
                    auto [_, art_lidx] = blid.get_articulation_id();
                    auto contact_frame = rtransform(cp.pos, mat3(cp.tangent1, cp.tangent2, cp.normal));
                    auto contact_rel_frame = art.get_global_joint_trans(art_lidx) / contact_frame;
                    glm::rvec3 lam_i = eigen_to_glm(lam.middleRows<3>(3*cid));
                    if (!sign) lam_i *= -1;
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
                    real lam_c = sign? lam_n(cid) : -lam_n(cid);
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
                real lam_c = sign? lam_n(cid) : -lam_n(cid);
                u.middleRows(dof_start, dof_size) += Minv_Jt.col(3*k+2) * (lam_c / cfg.dt);
            }
            if (bid.is_articulation()) {
                auto art_id = bid.get_art_id();
                auto& art = *get_articulated_body(art_id);
                integrate_implicit_euler(art.get_spec(), cfg.dt, nullptr,
                                         INOUT art.get_pos_buf(), INOUT u.data() + dof_start);
            }
            else {
                // TODO
            }
        }
    }

    auto t2 = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1);
    output_log("Contact solver: %lld ns\n", duration.count());


}

}
