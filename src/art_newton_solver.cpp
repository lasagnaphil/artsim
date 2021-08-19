//
// Created by lasagnaphil on 21. 7. 16..
//

/*
 * Implementation of "Non-Smooth Newton Methods for Deformable Multi-Body Dynamics" by Macklin et al.
 * This doesn't work yet, still in the process of debugging...
 */

#include <artsim/world.h>
#include <artsim/art_dynamics.h>
#include <artsim/math/eigen.h>

#include <Eigen/IterativeLinearSolvers>

namespace artsim {

struct StateDOFMetadata {
    struct Item {
        int pos_dof_starts;
        int pos_dofs;
        int vel_dof_starts;
        int vel_dofs;
    };
private:
    std::unordered_map<BodyId, Item> data;
    int num_total_pos_dofs = 0;
    int num_total_vel_dofs = 0;
    int num_bodies = 0;

public:
    void build(Arena<ArticulatedBody>& arts, Arena<RigidBody>& rbs) {
        int cur_pos_dof = 0;
        int cur_vel_dof = 0;
        arts.foreach_id_val([&](Id<ArticulatedBody> art_id, ArticulatedBody& art) {
            int pos_dof = art.get_num_pos_dofs();
            int vel_dof = art.get_num_vel_dofs();
            auto body_id = BodyId::from_articulated_body(art_id);
            this->data.insert({body_id, {cur_pos_dof, pos_dof, cur_vel_dof, vel_dof}});
            cur_pos_dof += pos_dof;
            cur_vel_dof += vel_dof;
            num_bodies++;
        });
        rbs.foreach_id_val([&](Id<RigidBody> rb_id, RigidBody& rb) {
            auto body_id = BodyId::from_rigid_body(rb_id);
            if (rb.spec.is_static) {
                this->data.insert({body_id, {cur_pos_dof, 0, cur_vel_dof, 0}});
            }
            else {
                this->data.insert({body_id, {cur_pos_dof, 7, cur_vel_dof, 6}});
                cur_pos_dof += 7;
                cur_vel_dof += 6;
            }
            num_bodies++;
        });
        this->num_total_pos_dofs = cur_pos_dof;
        this->num_total_vel_dofs = cur_vel_dof;
    }

    std::pair<int, int> get_pos_dof_starts_and_size(BodyId body_id) const {
        auto item = data.at(body_id);
        return {item.pos_dof_starts, item.pos_dofs};
    }

    std::pair<int, int> get_vel_dof_starts_and_size(BodyId body_id) const {
        auto item = data.at(body_id);
        return {item.vel_dof_starts, item.vel_dofs};
    }

    int get_total_vel_dof() const {
        return num_total_vel_dofs;
    }

    int get_total_pos_dof() const {
        return num_total_pos_dofs;
    }

    int get_num_bodies() const {
        return num_bodies;
    }
};

void World::newton_solver() {
    int num_contact_points = contact_points.size();

    StateDOFMetadata state_meta;
    state_meta.build(articulated_bodies, rigid_bodies);

    int total_pos_dof = state_meta.get_total_pos_dof();
    int total_vel_dof = state_meta.get_total_vel_dof();
    int num_bodies = state_meta.get_num_bodies();

    // Mass matrix and its inverse
    MatrixXr M(total_vel_dof, total_vel_dof);
    MatrixXr Minv(total_vel_dof, total_vel_dof);
    M.setZero();
    Minv.setZero();

    // System matrix
    MatrixXr A(3*num_contact_points, 3*num_contact_points);
    A.setZero();

    // Jacobian
    MatrixXr J_c(3*num_contact_points, total_vel_dof);
    J_c.setZero();

    // Compliance matrix (diagonal components)
    VectorXr C(3*num_contact_points);
    C.setZero();

    // Initial position
    VectorXr q0(total_pos_dof);

    // Velocity
    VectorXr u(total_vel_dof);
    VectorXr u_tilde(total_vel_dof);
    VectorXr du(total_vel_dof);

    // Lagrange multipliers and its increments
    VectorXr lam(3*num_contact_points);
    VectorXr dlam(3*num_contact_points);

    // Right-side vectors
    // VectorXr g(total_vel_dof);
    VectorXr h(3*num_contact_points);
    VectorXr b(3*num_contact_points);

    // Preconditioning for constraints
    VectorXr r(3*num_contact_points);

    // Contact constraint function
    VectorXr c_n(num_contact_points);

    // Material properties for each constraint
    std::vector<Material> materials(num_contact_points);

    // Initialize u_tilde & mass matrix
    articulated_bodies.foreach_id_val([&](Id<ArticulatedBody> art_id, ArticulatedBody& art){
        auto bid = BodyId::from_articulated_body(art_id);
        auto [pos_dof_start, pos_dofs] = state_meta.get_pos_dof_starts_and_size(bid);
        auto [vel_dof_start, vel_dofs] = state_meta.get_vel_dof_starts_and_size(bid);
        std::copy_n(art.get_pos_buf(), pos_dofs, OUT q0.data() + pos_dof_start);
        glmx::dynmat<real> M_art(vel_dofs, vel_dofs);
        art.mass_matrix(OUT M_art.to_view(), cfg.dt);
        M.block(vel_dof_start, vel_dof_start, vel_dofs, vel_dofs) = Eigen::Map<MatrixXr>(M_art.data(), vel_dofs, vel_dofs);
        art.forward_dynamics(cfg.gravity, cfg.dt);
        real* art_u = art.get_vel_buf();
        real* art_udot = art.get_acc_buf();
        for (int k = 0; k < vel_dofs; k++) {
            u_tilde(vel_dof_start + k) = art_u[k] + cfg.dt * art_udot[k];
        }
    });
    rigid_bodies.foreach_id_val([&](Id<RigidBody> rb_id, RigidBody& rb) {
        if (rb.spec.is_static) return;
        auto bid = BodyId::from_rigid_body(rb_id);
        auto [dof_start, dofs] = state_meta.get_vel_dof_starts_and_size(bid);
        // TODO
    });
    Minv = M.inverse();

    // Initialize u, lam
    u.setZero();
    // u = u_tilde;
    lam.setZero();

    printf("Newton step start\n");
    const int max_newton_iters = 20;
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

            r(3*cidx+0) = r(3*cidx+1) = cfg.dt;
            r(3*cidx+2) = cfg.dt * cfg.dt;
            c_n(cidx) = glm::dot(cp.normal, pos1 - pos2);
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
                auto J_view = glmx::dynmat_view<real>((real*)J.data(), 3, num_art_vel_dofs);
                auto contact_frame = glm::rmat3(cp.tangent1, cp.tangent2, cp.normal);
                calc_linear_jacobian(art_spec, lidx,
                                     glmx::rtransform(cp.pos, contact_frame),
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
            std::vector<glm::rvec3> J1 = calc_body_jac(cp, cp.body1_id);
            std::vector<glm::rvec3> J2 = calc_body_jac(cp, cp.body2_id);
            auto [body1_dof_start, body1_dofs] = state_meta.get_vel_dof_starts_and_size(bid1);
            auto [body2_dof_start, body2_dofs] = state_meta.get_vel_dof_starts_and_size(bid2);
            glm::rvec2 v_f = glm::rvec2(0);
            for (int k = 0; k < body1_dofs; k++) {
                int sidx = body1_dof_start + k;
                v_f.x += J1[k].x * u(sidx);
                v_f.y += J1[k].y * u(sidx);
            }
            for (int k = 0; k < body2_dofs; k++) {
                int sidx = body2_dof_start + k;
                v_f.x -= J2[k].x * u(sidx);
                v_f.y -= J2[k].y * u(sidx);
            }

            const real epsilon = 1e-9;
            real r_n = r(3*cidx+2);
            real lam_n = lam(3*cidx+2);
            real r_n_lam_n = r_n * lam_n;
            real C_n = c_n(cidx);
            real size_n = glm::sqrt(SQR(C_n) + SQR(r_n_lam_n));
            real alpha, beta;
            if (glm::abs(C_n) < epsilon && glm::abs(r_n_lam_n) < epsilon) {
                alpha = real(0);
                beta = real(1);
            }
            else {
                alpha = real(1) - C_n / size_n;
                beta = real(1) - r_n_lam_n / size_n;
            }
            real phi_n = C_n + r_n_lam_n - size_n;

            // Note: how should we set the preconditioner? Is a min() okay?
            real r_f = glm::min(r(3*cidx+0), r(3*cidx+1));
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

            for (int k = 0; k < body1_dofs; k++) {
                int sidx = body1_dof_start + k;
                J_c(3*cidx+0, sidx) += J1[k].x;
                J_c(3*cidx+1, sidx) += J1[k].y;
                J_c(3*cidx+2, sidx) += alpha * glm::dot(J1[k], cp.normal);
            }
            for (int k = 0; k < body2_dofs; k++) {
                int sidx = body2_dof_start + k;
                J_c(3*cidx+0, sidx) -= J2[k].x;
                J_c(3*cidx+1, sidx) -= J2[k].y;
                J_c(3*cidx+2, sidx) -= alpha * glm::dot(J2[k], cp.normal);
            }

            C(3*cidx+0) = C(3*cidx+1) = (lam_n > 0? W : r_f) / cfg.dt;
            C(3*cidx+2) = beta * r_n / SQR(cfg.dt);

            h(3*cidx+0) = phi_f.x;
            h(3*cidx+1) = phi_f.y;
            h(3*cidx+2) = phi_n / cfg.dt;
        }

        VectorXr g = M * (u - u_tilde) - J_c.transpose() * (cfg.dt * lam);
        A = J_c * Minv * J_c.transpose();
        A.diagonal() += C;
        b = (J_c * (Minv * g) - h) / cfg.dt;
        Eigen::ConjugateGradient<MatrixXr, Eigen::Lower|Eigen::Upper, Eigen::DiagonalPreconditioner<real>> cg;
        cg.setMaxIterations(20);
        cg.setTolerance(1e-6);
        cg.compute(A);
        dlam = cg.solve(b);
        du = Minv * (J_c.transpose() * (cfg.dt * dlam) - g);

        lam += real(0.75) * dlam;
        u += real(0.75) * du;

        for (int cidx = 0; cidx < num_contact_points; cidx++) {
            r(3*cidx) = cfg.dt * A(3*cidx, 3*cidx);
            r(3*cidx+1) = cfg.dt * A(3*cidx+1, 3*cidx+1);
            r(3*cidx+2) = cfg.dt * cfg.dt * A(3*cidx+2, 3*cidx+2);
        }

        articulated_bodies.foreach_id_val([&](Id<ArticulatedBody> art_id, ArticulatedBody& art){
            auto bid = BodyId::from_articulated_body(art_id);
            auto [pos_dof_start, pos_dofs] = state_meta.get_pos_dof_starts_and_size(bid);
            auto [vel_dof_start, vel_dofs] = state_meta.get_vel_dof_starts_and_size(bid);
            std::copy_n(q0.data() + pos_dof_start, pos_dofs, OUT art.get_pos_buf());
            std::copy_n(u.data() + vel_dof_start, vel_dofs, OUT art.get_vel_buf());
            integrate_positions(art.get_spec(), cfg.dt, art.get_vel_buf(), art.get_pos_buf());
        });
        rigid_bodies.foreach_id_val([&](Id<RigidBody> rb_id, RigidBody& rb) {
            // TODO
        });

        std::cout << std::endl;
        printf("||dlam|| = %f, ||du|| = %f, ||h|| = %f\n", dlam.norm(), du.norm(), h.norm());
        std::cout << "dlam:\t" << dlam.transpose() << std::endl;
        std::cout << "lam:\t" << lam.transpose() << std::endl;
        std::cout << "du:\t" << du.transpose() << std::endl;
        std::cout << "u:\t" << u.transpose() << std::endl;
        std::cout << "g:\t" << g.transpose() << std::endl;
        std::cout << "h:\t" << h.transpose() << std::endl;
        std::cout << "c_n:\t" << c_n.transpose() << std::endl;

        std::cout << "A:" << std::endl;
        std::cout << A << std::endl;
        std::cout << "b:\t" << b.transpose() << std::endl;
        std::cout << "J:" << std::endl;
        std::cout << J_c << std::endl;
    }

}

}
