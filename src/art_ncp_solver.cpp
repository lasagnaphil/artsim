//
// Created by lasagnaphil on 21. 7. 16..
//

#include "artsim/art_ncp_solver.h"
#include "artsim/art_dynamics.h"
#include "artsim/math/eigen.h"

namespace artsim {

struct StateDOFMetadata {
    struct Item {
        int vel_dof_starts;
        int vel_dofs;
        int pos_dof_starts;
        int pos_dofs;
    };
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

    // Initialize u_tilde & preintegrate bodies
    articulated_bodies.foreach_id_val([&](Id<ArticulatedBody> art_id, ArticulatedBody& art){
        auto bid = BodyId::from_articulated_body(art_id);
        auto [dof_start, dofs] = state_meta.get_dof_starts_and_size(bid);
        Eigen::Map<VectorXr> u0(art.get_vel_buf(), dofs);
        art.forward_dynamics(cfg.gravity, cfg.dt);
        art.integrate(cfg.dt);
        std::copy_n(art.get_vel_buf(), dofs, OUT u_tilde.data() + dof_start);
    });
    rigid_bodies.foreach_id_val([&](Id<RigidBody> rb_id, RigidBody& rb) {
        if (rb.spec.is_static) return;
        auto bid = BodyId::from_rigid_body(rb_id);
        int dof_start = state_meta.get_dof_starts(bid);
        // TODO
    });
    // Initialize u, lam
    u.setZero();
    // u = u_tilde;
    lam.setZero();

    printf("Newton step start\n");
    const int max_newton_iters = 10;
    for (int iter = 0; iter < max_newton_iters; iter++) {
        for (int cidx = 0; cidx < num_contact_points; cidx++) {
            glm::rvec3 pos1, pos2;
            Id<Material> mat1, mat2;

            auto& cp = contact_points[cidx];
            if (cp.body1_id.is_articulation()) {
                auto [art_id, lidx] = cp.body1_id.get_articulation_id();
                auto art = get_articulated_body(art_id);
                pos1 = (art->get_global_joint_trans(lidx) * cp.body1_rel_trans).v;
                mat1 = art->get_mat_id();
            }
            else {
                auto rb = get_rigid_body(cp.body1_id.get_rigid_body_id());
                pos1 = (glmx::rtransform(rb->pos, glm::mat3_cast(rb->rot)) * cp.body1_rel_trans).v;
                mat1 = rb->mat_id;
            }
            if (cp.body2_id.is_articulation()) {
                auto [art_id, lidx] = cp.body2_id.get_articulation_id();
                auto art = get_articulated_body(art_id);
                pos2 = (art->get_global_joint_trans(lidx) * cp.body2_rel_trans).v;
                mat2 = art->get_mat_id();
            }
            else {
                auto rb = get_rigid_body(cp.body2_id.get_rigid_body_id());
                pos2 = (glmx::rtransform(rb->pos, glm::mat3_cast(rb->rot)) * cp.body2_rel_trans).v;
                mat2 = rb->mat_id;
            }

            c_n(cidx) = glm::dot(cp.normal, pos1 - pos2);
            materials[cidx] = material_db.get_material_pair(mat1, mat2);
        }

#define SQR(x) ((x)*(x))

        auto calc_body_jac = [&](const ContactPoint& cp, BodyLinkId bid, glm::rvec3 pos) -> std::vector<glm::rvec3> {
            std::vector<glm::rvec3> J;
            if (bid.is_articulation()) {
                auto [art_id, lidx] = bid.get_articulation_id();
                auto art = get_articulated_body(art_id);
                auto& art_spec = art->get_spec();
                int num_art_vel_dofs = art->get_num_vel_dofs();
                J.resize(num_art_vel_dofs);
                auto J_view = glmx::dynmat_view<real>((real*)J.data(), 3, num_art_vel_dofs);
                auto contact_frame = glm::rmat3(cp.tangent1, cp.tangent2, cp.normal);
                calc_linear_jacobian(art_spec, lidx,
                                     glmx::rtransform(pos, contact_frame) / art->get_global_joint_trans(lidx),
                                     art->get_global_joint_trans_buf(), OUT J_view);
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
        C.setZero();
        h.setZero();

        for (int cidx = 0; cidx < num_contact_points; cidx++) {
            auto& cp = contact_points[cidx];
            BodyId bid1 = cp.body1_id.get_body_id();
            BodyId bid2 = cp.body2_id.get_body_id();
            std::vector<glm::rvec3> J1 = calc_body_jac(cp, cp.body1_id, cp.pos);
            std::vector<glm::rvec3> J2 = calc_body_jac(cp, cp.body2_id, cp.pos);
            int body1_dof_start = state_meta.get_dof_starts(bid1);
            int body2_dof_start = state_meta.get_dof_starts(bid2);
            glm::rvec2 v_f = glm::rvec2(0);
            for (int k = 0; k < J1.size(); k++) {
                int sidx = body1_dof_start + k;
                v_f.x += J1[k].x * u(sidx);
                v_f.y += J1[k].y * u(sidx);
            }
            for (int k = 0; k < J2.size(); k++) {
                int sidx = body2_dof_start + k;
                v_f.x -= J2[k].x * u(sidx);
                v_f.y -= J2[k].y * u(sidx);
            }

            const real epsilon = 1e-9;
            real r_n = cfg.dt*cfg.dt; // TODO: use a better preconditioner
            real lam_n = lam(3*cidx);
            real r_n_lam_n = r_n * lam_n;
            real C_n = c_n(cidx);
            // if (C_n > 0 && lam_n == 0) continue;
            real size_n = glm::sqrt(SQR(C_n) + SQR(r_n_lam_n));
            real alpha, beta;
            if (glm::abs(C_n) < epsilon && glm::abs(r_n_lam_n) < epsilon) {
                alpha = 0;
                beta = 1;
            }
            else {
                alpha = 1 - C_n / size_n;
                beta = 1 - r_n_lam_n / size_n;
            }
            real phi_n = C_n + r_n_lam_n - size_n;

            real r_f = cfg.dt; // TODO: use a better preconditioner
            real v_f_len_sq = glm::length2(v_f);
            real v_f_len = glm::sqrt(v_f_len_sq);
            real mu = materials[cidx].friction;
            glm::rvec2 lam_f = glm::make_vec2(lam.data() + 3*cidx);
            real lam_f_len = glm::length(lam_f);
            real lam_res = r_f*(mu*lam_n - lam_f_len);
            real size_f = glm::sqrt(v_f_len_sq + SQR(lam_res));
            real W = r_f * (size_f - lam_res) / (v_f_len + r_f*mu*lam_n - size_f);
            if (glm::isnan(W) || glm::isinf(W)) {
                W = r_f;
            }
            glm::rvec2 phi_f = v_f + W * lam_f;

            for (int k = 0; k < J1.size(); k++) {
                int sidx = body1_dof_start + k;
                J_c(3*cidx+0, sidx) += J1[k].x;
                J_c(3*cidx+1, sidx) += J1[k].y;
                J_c(3*cidx+2, sidx) += alpha * glm::dot(J1[k], cp.normal);
            }
            for (int k = 0; k < J2.size(); k++) {
                int sidx = body2_dof_start + k;
                J_c(3*cidx+0, sidx) -= J2[k].x;
                J_c(3*cidx+1, sidx) -= J2[k].y;
                J_c(3*cidx+2, sidx) -= alpha * glm::dot(J2[k], cp.normal);
            }

            C(3*cidx+0) = C(3*cidx+1) = (lam_n > 0? W : 1) / cfg.dt;
            C(3*cidx+2) = beta / SQR(cfg.dt);

            h(3*cidx+0) = phi_f.x;
            h(3*cidx+1) = phi_f.y;
            h(3*cidx+2) = phi_n / cfg.dt;
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
        if (dlam.norm() < 1e-6) break;

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
        if (du.norm() < 1e-6) break;
        VectorXr t_du = real(0.75) * du;

        lam += real(0.75) * dlam;
        u += t_du;

        articulated_bodies.foreach_id_val([&](Id<ArticulatedBody> art_id, ArticulatedBody& art){
            auto bid = BodyId::from_articulated_body(art_id);
            auto [dof_start, dofs] = state_meta.get_dof_starts_and_size(bid);
            std::copy_n(t_du.data() + dof_start, dofs, OUT art.get_vel_buf());
            art.integrate(cfg.dt, false);
        });
        rigid_bodies.foreach_id_val([&](Id<RigidBody> rb_id, RigidBody& rb) {
            // TODO
        });
        printf("||dlam|| = %f, ||du|| = %f, ||h|| = %f\n", dlam.norm(), du.norm(), h.norm());
        std::cout << "lam: " << lam.transpose() << std::endl;
        std::cout << "u: " << u.transpose() << std::endl;
        std::cout << "h: " << h.transpose() << std::endl;
    }

}

}
