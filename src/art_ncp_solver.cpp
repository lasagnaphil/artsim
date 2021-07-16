//
// Created by lasagnaphil on 21. 7. 16..
//

#include "artsim/art_ncp_solver.h"
#include "artsim/art_dynamics.h"
#include "artsim/math/eigen.h"

namespace artsim {

struct StateDOFMetadata {
private:
    std::unordered_map<BodyId, std::pair<int, int>> data;
    int num_total_dofs = 0;
    int num_bodies = 0;

public:
    void build(Arena<ArticulatedBody>& arts, Arena<RigidBody>& rbs) {
        int cur_dof = 0;
        arts.foreach_id_val([&](Id<ArticulatedBody> art_id, ArticulatedBody& art) {
            int dof = art.get_num_vel_dofs();
            auto body_id = BodyId::from_articulated_body(art_id);
            this->data.insert({body_id, std::make_pair(cur_dof, dof)});
            cur_dof += dof;
            num_bodies++;
        });
        rbs.foreach_id_val([&](Id<RigidBody> rb_id, RigidBody& rb) {
            auto body_id = BodyId::from_rigid_body(rb_id);
            if (rb.spec.is_static) {
                this->data.insert({body_id, std::make_pair(cur_dof, 0)});
            }
            else {
                this->data.insert({body_id, std::make_pair(cur_dof, 6)});
                cur_dof += 6;
                num_bodies++;
            }
        });
        this->num_total_dofs = cur_dof;
    }

    std::pair<int, int> get_dof_starts_and_size(BodyId body_id) const {
        return data.at(body_id);
    }

    int get_dof_starts(BodyId body_id) const {
        return data.at(body_id).first;
    }

    int get_dof_size(BodyId body_id) const {
        return data.at(body_id).second;
    }

    int get_total_dof() const {
        return num_total_dofs;
    }

    int get_num_bodies() const {
        return num_bodies;
    }
};

void World::ncp_solver(const ContactPoint* contact_points, int num_contact_points) {
    StateDOFMetadata state_meta;
    state_meta.build(articulated_bodies, rigid_bodies);

    int total_dof = state_meta.get_total_dof();
    int num_bodies = state_meta.get_num_bodies();

    // Mass matrices and its inverses
    std::vector<MatrixXr> M(num_bodies);
    std::vector<MatrixXr> Minv(num_bodies);

    // System matrix
    MatrixXr A(3*num_contact_points, 3*num_contact_points);
    A.setZero();

    // Jacobian
    MatrixXr J_c(3*num_contact_points, total_dof);
    J_c.setZero();

    // Compliance matrix (diagonal components)
    VectorXr C(3*num_contact_points);
    C.setZero();

    // Velocity
    VectorXr u(total_dof);
    VectorXr u_tilde(total_dof);
    VectorXr du(total_dof);

    // Lagrange multipliers and its increments
    VectorXr lam(3*num_contact_points);
    VectorXr dlam(3*num_contact_points);

    // Right-side vectors
    // VectorXr g(total_dof);
    VectorXr h(3*num_contact_points);
    VectorXr b(3*num_contact_points);

    // Contact constraint function
    VectorXr c_n(num_contact_points);

    // Material properties for each constraint
    std::vector<Material> materials(num_contact_points);

    articulated_bodies.foreach_id_val([&](Id<ArticulatedBody> art_id, ArticulatedBody& art){
        auto bid = BodyId::from_articulated_body(art_id);
        int dof_start = state_meta.get_dof_starts(bid);
        featherstone_forward_dynamics(art.get_spec(), cfg.gravity, cfg.dt,
                                      art.get_external_force_buf(),
                                      art.get_pos_buf(), art.get_vel_buf(),
                                      art.get_internal_force_buf(),
                                      art.get_target_pos_buf(),
                                      OUT u_tilde.data() + dof_start);
    });
    rigid_bodies.foreach_id_val([&](Id<RigidBody> rb_id, RigidBody& rb) {
        if (rb.spec.is_static) return;
        auto bid = BodyId::from_rigid_body(rb_id);
        int dof_start = state_meta.get_dof_starts(bid);
        // TODO
    });

    const int max_newton_iters = 5;
    for (int iter = 0; iter < max_newton_iters; iter++) {
        for (int cidx = 0; cidx < num_contact_points; cidx++) {
            auto calc_prerequisites = [&](const ContactPoint& cp, BodyLinkId bid,
                                          OUT glm::rvec3& pos, OUT Id<Material>& mat) {
                if (bid.is_articulation()) {
                    auto [art_id, lidx] = bid.get_articulation_id();
                    auto art = get_articulated_body(art_id);
                    pos = (art->get_global_joint_trans(lidx) * cp.body1_rel_trans).v;
                    mat = art->get_mat_id();
                }
                else {
                    auto rb = get_rigid_body(bid.get_rigid_body_id());
                    pos = (glmx::rtransform(rb->pos, glm::mat3_cast(rb->rot)) * cp.body1_rel_trans).v;
                    mat = rb->mat_id;
                }
            };

            glm::rvec3 pos1, pos2;
            Id<Material> mat1, mat2;
            auto& cp = contact_points[cidx];
            calc_prerequisites(cp, cp.body1_id, OUT pos1, OUT mat1);
            calc_prerequisites(cp, cp.body2_id, OUT pos2, OUT mat2);

            c_n(cidx) = glm::dot(cp.normal, pos1 - pos2) - cp.depth;
            materials[cidx] = material_db.get_material_pair(mat1, mat2);
        }

#define SQR(x) ((x)*(x))

        auto calc_body_jac = [&](const ContactPoint& cp, BodyLinkId bid) -> std::vector<glm::rvec3> {
            std::vector<glm::rvec3> J;
            if (bid.is_articulation()) {
                auto [art_id, lidx] = bid.get_articulation_id();
                auto art = get_articulated_body(art_id);
                auto& art_spec = art->get_spec();
                int num_art_vel_dofs = art->get_num_vel_dofs();
                J.resize(num_art_vel_dofs);
                glmx::dynmat_view<real> J_view((real*)J.data(), 3, num_art_vel_dofs);
                calc_linear_jacobian(art_spec, lidx,
                                     glmx::rtransform(cp.pos) / art->get_global_joint_trans(lidx),
                                     art->get_global_joint_trans_buf(), J_view);
            }
            else if (bid.is_rigid_body()) {
                auto sb_id = bid.get_rigid_body_id();
                auto rb = get_rigid_body(sb_id);
                if (rb->spec.is_static) {
                    return J;
                }
                // TODO
            }
            return J;
        };

        J_c.setZero();
        for (int cidx = 0; cidx < num_contact_points; cidx++) {
            auto& cp = contact_points[cidx];
            BodyId bid1 = cp.body1_id.get_body_id();
            BodyId bid2 = cp.body2_id.get_body_id();
            std::vector<glm::rvec3> J1 = calc_body_jac(cp, cp.body1_id);
            std::vector<glm::rvec3> J2 = calc_body_jac(cp, cp.body2_id);
            int body1_dof_start = state_meta.get_dof_starts(bid1);
            int body2_dof_start = state_meta.get_dof_starts(bid2);
            glm::rvec2 v_f = glm::rvec2(0);
            for (int k = 0; k < J1.size(); k++) {
                v_f.x += J1[k].x * u(k);
                v_f.y += J1[k].y * u(k);
            }
            for (int k = 0; k < J2.size(); k++) {
                v_f.x -= J2[k].x * u(k);
                v_f.y -= J2[k].x * u(k);
            }

            real r_n = cfg.dt*cfg.dt; // TODO: use a better preconditioner
            real lam_n = lam(3*cidx);
            real r_n_lam_n = r_n * lam_n;
            real C_n = c_n(cidx);
            real size_n = glm::sqrt(SQR(C_n) + SQR(r_n_lam_n));
            real alpha = 1 - C_n / size_n;
            real beta = 1 - r_n_lam_n / size_n;
            real phi_n = C_n + r_n_lam_n - size_n;

            real r_f = cfg.dt; // TODO: use a better preconditioner
            real v_f_len_sq = glm::length2(v_f);
            real v_f_len = glm::sqrt(v_f_len_sq);
            real mu = materials[cidx].friction;
            glm::rvec2 lam_f = glm::make_vec2(lam.data() + 3*cidx + 1);
            real lam_f_len = glm::length(lam_f);
            real lam_res = r_f*(mu*lam_n - lam_f_len);
            real size_f = glm::sqrt(v_f_len_sq + SQR(lam_res));
            real W = r_f * (size_f - lam_res) / (v_f_len + r_f*mu*lam_n - size_f);
            glm::rvec2 phi_f = v_f + W * lam_f;

            for (int k = 0; k < J1.size(); k++) {
                int sidx = body1_dof_start + k;
                J_c(3*cidx+0, sidx) += alpha * glm::dot(J1[k], cp.normal);
                J_c(3*cidx+1, sidx) += J1[k].x;
                J_c(3*cidx+2, sidx) += J1[k].y;
            }
            for (int k = 0; k < J2.size(); k++) {
                int sidx = body2_dof_start + k;
                J_c(3*cidx+0, sidx) -= alpha * glm::dot(J2[k], cp.normal);
                J_c(3*cidx+1, sidx) -= J2[k].x;
                J_c(3*cidx+2, sidx) -= J2[k].y;
            }

            C(3*cidx+0) = beta;
            C(3*cidx+1) = C(3*cidx+2) = (lam_n > 0? W : 1);

            h(3*cidx+0) = phi_n / cfg.dt;
            h(3*cidx+1) = phi_f.x;
            h(3*cidx+2) = phi_f.y;
        }

        A.setZero();
        articulated_bodies.foreach_id_val([&](Id<ArticulatedBody> art_id, ArticulatedBody& art){
            auto bid = BodyId::from_articulated_body(art_id);
            auto [dof_start, dofs] = state_meta.get_dof_starts_and_size(bid);
            MatrixXr Jt = J_c.middleCols(dof_start, dofs).transpose();
            MatrixXr Minv_Jt(dofs, 3*num_contact_points);
            multiply_inverse_mass_matrix(art.get_spec(), cfg.dt, art.get_pos_buf(), get_view(Jt),
                                         OUT get_view(Minv_Jt));
            A += Jt.transpose() * Minv_Jt;
        });
        rigid_bodies.foreach_id_val([&](Id<RigidBody> rb_id, RigidBody& rb) {
            // TODO
        });

        b = (real(1)/cfg.dt) * (J_c * (u - u_tilde) - h) - A * lam;
        A.diagonal() += C;

        // TODO: solve this using Conjugate Residual
        dlam = A.ldlt().solve(b);

        VectorXr Jt_lam_prime = J_c.transpose() * (lam + dlam);
        VectorXr Minv_Jt_lam_prime(total_dof);
        articulated_bodies.foreach_id_val([&](Id<ArticulatedBody> art_id, ArticulatedBody& art){
            auto bid = BodyId::from_articulated_body(art_id);
            auto [dof_start, dofs] = state_meta.get_dof_starts_and_size(bid);
            multiply_inverse_mass_matrix(art.get_spec(), cfg.dt, art.get_pos_buf(),
                                         Jt_lam_prime.data() + dof_start, OUT Minv_Jt_lam_prime.data() + dof_start);
        });
        rigid_bodies.foreach_id_val([&](Id<RigidBody> rb_id, RigidBody& rb) {
            // TODO
        });
        du = u_tilde - u + cfg.dt * Minv_Jt_lam_prime;

        lam += real(0.75) * dlam;
        u += real(0.75) * du;

        articulated_bodies.foreach_id_val([&](Id<ArticulatedBody> art_id, ArticulatedBody& art){
            auto bid = BodyId::from_articulated_body(art_id);
            auto [dof_start, dofs] = state_meta.get_dof_starts_and_size(bid);
            std::copy_n(u.data() + dof_start, dofs, OUT art.get_vel_buf());
            art.integrate(cfg.dt, false);
        });
        rigid_bodies.foreach_id_val([&](Id<RigidBody> rb_id, RigidBody& rb) {
            // TODO
        });

    }

}

}
