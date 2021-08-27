//
// Created by lasagnaphil on 21. 7. 20..
//

#include "artsim/soft_body_dynamics.h"
#include "artsim/math/fastsvd.h"

#include <deque>
#include <fmt/core.h>

namespace artsim {

using namespace Eigen;
using MatrixXr = Eigen::Matrix<real, Eigen::Dynamic, Eigen::Dynamic>;
using VectorXr = Eigen::Matrix<real, Eigen::Dynamic, 1>;

real energy_eigvec(glm::rvec3 S, const ARAPEnergyConstraint& c) {
    glm::rvec3 dS = S - glm::rvec3(1);
    return c.mu * glm::length2(dS);
}

real energy_eigvec(glm::tvec3<real> S, const CorotationalEnergyConstraint& c) {
    glm::rvec3 dS = S - glm::rvec3(1);
    real tr_dS = dS[0] + dS[1] + dS[2];
    return c.mu * glm::length2(dS) + (c.lambda/2) * tr_dS * tr_dS;
}

real energy_eigvec(glm::tvec3<real> S, const NeoHookeanEnergyConstraint& c) {
    glm::rvec3 S_pow2 = S * S;
    real I1 = S[0]*S[0] + S[1]*S[1] + S[2]*S[2];
    real J = S[0]*S[1]*S[2];
    real log_J = glm::log(J);
    return (c.mu/2) * (I1 - 2*log_J - 3) + (c.lambda/2)*log_J*log_J;
}

glm::rmat3 pk1(const glm::rmat3& F, const glmx::SVD_mats<real>& F_svd, const ARAPEnergyConstraint& c) {
    glm::rmat3 R = F_svd.U * glm::transpose(F_svd.V);
    glm::rvec3 dS = glm::rvec3(F_svd.Sigma[0]-1, F_svd.Sigma[1]-1, F_svd.Sigma[2]-1);
    return 2*c.mu*glmx::svd_mult(F_svd.U, dS, F_svd.V);
}

glm::rmat3 pk1(const glm::rmat3& F, const glmx::SVD_mats<real>& F_svd, const CorotationalEnergyConstraint& c) {
    glm::rmat3 R = F_svd.U * glm::transpose(F_svd.V);
    glm::rvec3 dS = glm::rvec3(F_svd.Sigma[0]-1, F_svd.Sigma[1]-1, F_svd.Sigma[2]-1);
    return 2*c.mu*glmx::svd_mult(F_svd.U, dS, F_svd.V) + c.lambda*glmx::tr(glmx::svd_mult(F_svd.V, F_svd.Sigma, F_svd.V))*R;
}

glm::rmat3 pk1(const glm::rmat3& F, const glmx::SVD_mats<real>& F_svd, const NeoHookeanEnergyConstraint& c) {
    real J = F_svd.Sigma[0] * F_svd.Sigma[1] * F_svd.Sigma[2];
    glm::rmat3 F_inv_T = F_svd.recover_inverse_transpose_matrix();
    return c.mu*(F - F_inv_T) + c.lambda*glm::log(J)*F_inv_T;
}

glm::tvec3<real> projection_eigvec(glm::tvec3<real> S, const LinearStrainEnergyConstraint& c) {
    S = glm::clamp(S, c.sigma_min, c.sigma_max);
    // Flip last singular value if determinant is negative
    if (S[0] * S[1] * S[2] < 0) {
        S[2] = -S[2];
    }
    return S;
}

glm::tvec3<real> projection_eigvec(glm::tvec3<real> S, const VolumePreservationEnergyConstraint& c) {
    real sigma = S[0]*S[1]*S[2];
    if (sigma < c.sigma_min) sigma = c.sigma_min;
    if (sigma > c.sigma_max) sigma = c.sigma_max;
    else return S;
    glm::tvec3<real> D(0);
    for (int i = 0; i < 5; i++) {
        glm::tvec3<real> S_star = S + D;
        real C = S_star[0]*S_star[1]*S_star[2] - sigma;
        glm::tvec3<real> grad_C = glm::tvec3<real>(S_star[1]*S_star[2], S_star[2]*S_star[0], S_star[0]*S_star[1]);
        D = ((glm::dot(grad_C, D) - C) / glm::length2(grad_C)) * grad_C;
    }
    return S + D;
}

template <class Constraint>
glm::tmat3x3<real> projection(const glm::tmat3x3<real>& F, const Constraint& c) {
    auto F_svd = glmx::svd(F);
    F_svd.Sigma = projection_eigvec(F_svd.Sigma, c);
    auto Sigma = glm::tmat3x3<real>(F_svd.Sigma[0], 0, 0, 0, F_svd.Sigma[1], 0, 0, 0, F_svd.Sigma[2]);
    return F_svd.U * Sigma * glm::transpose(F_svd.V);
}

glm::rvec3 proximal_eigvec(glm::rvec3 sigma, real volume, const ARAPEnergyConstraint& c) {
    real tau = c.k / volume;
    return glm::rvec3(
            (2*c.mu + tau * sigma.x) / (2*c.mu + tau),
            (2*c.mu + tau * sigma.y) / (2*c.mu + tau),
            (2*c.mu + tau * sigma.z) / (2*c.mu + tau));
}

glm::tvec3<real> proximal_eigvec(glm::tvec3<real> sigma, real volume, const CorotationalEnergyConstraint& c) {
    real tau = c.k / volume;
    real A_diag = 2*c.mu + c.lambda + tau;
    glmx::tsmat3x3<real> A(A_diag, A_diag, A_diag, c.lambda, c.lambda, c.lambda);
    glm::tvec3<real> b(2*c.mu + 3*c.lambda + tau*sigma.x,
                       2*c.mu + 3*c.lambda + tau*sigma.y,
                       2*c.mu + 3*c.lambda + tau*sigma.z);
    return inverse(A) * b;
}

// TODO: this explodes because one of the components of S becomes zero.
glm::tvec3<real> proximal_eigvec(glm::tvec3<real> sigma, real volume, const NeoHookeanEnergyConstraint& c) {
    auto S = sigma;
    real tau = c.k / volume;
    for (int i = 0; i < 5; i++) {
        real J = log(S[0]*S[1]*S[2]);
        glm::tvec3<real> grad;
        grad[0] = c.mu*(S[0] - 1.0/S[0]) + c.lambda/S[0] * J + tau*(S[0] - sigma[0]);
        grad[1] = c.mu*(S[1] - 1.0/S[1]) + c.lambda/S[1] * J + tau*(S[1] - sigma[1]);
        grad[2] = c.mu*(S[2] - 1.0/S[2]) + c.lambda/S[2] * J + tau*(S[2] - sigma[2]);
        glmx::tsmat3x3<real> H;
        H.xx = c.mu + (c.mu + c.lambda)/(S[0]*S[0]) - c.lambda/(S[0]*S[0]) * J + tau;
        H.yy = c.mu + (c.mu + c.lambda)/(S[1]*S[1]) - c.lambda/(S[1]*S[1]) * J + tau;
        H.zz = c.mu + (c.mu + c.lambda)/(S[2]*S[2]) - c.lambda/(S[2]*S[2]) * J + tau;
        H.yz = c.lambda / (S[1]*S[2]);
        H.zx = c.lambda / (S[2]*S[0]);
        H.xy = c.lambda / (S[0]*S[1]);
        /*
        std::cout << glm::to_string(S) << std::endl;
        if (glm::isnan(H.xx) || glm::isnan(H.yy) || glm::isnan(H.zz)) {
            std::cout << "Nan detected!" << std::endl;
            exit(EXIT_FAILURE);
        }
        if (glm::epsilonEqual(glmx::determinant(H), 0., 1e-8)) {
            std::cout << "Singular matrix!" << std::endl;
            exit(EXIT_FAILURE);
        }
         */
        S -= glmx::inverse(H) * grad;
        S = glm::max(S, glm::rvec3(0)); // Prevent volume from becoming negative
    }
    return S;
}

template <class Constraint>
glm::tmat3x3<real> proximal(const glm::tmat3x3<real>& F, real volume, const Constraint& c) {
    auto F_svd = glmx::svd(F);
    F_svd.Sigma = proximal_eigvec(F_svd.Sigma, volume, c);
    glm::tmat3x3<real> Sigma(F_svd.Sigma.x, 0, 0, 0, F_svd.Sigma.y, 0, 0, 0, F_svd.Sigma.z);
    return F_svd.U * Sigma * glm::transpose(F_svd.V);
}

void soft_body_calc_deformation_field(const SoftBody& body, const glm::rvec3* x, OUT glm::rmat3* F) {
    int num_tets = body.tets.size();
    for (int tidx = 0; tidx < num_tets; tidx++) {
        auto& tet = body.tets[tidx];
        F[tidx] = glm::rmat3(x[tet[0]] - x[tet[3]], x[tet[1]] - x[tet[3]], x[tet[2]] - x[tet[3]]) * body.B_m[tidx];
    }
}

void soft_body_calc_deformation_field(const SoftBody& body, const glm::rvec3* x, const glm::rmat3* u, OUT glm::rmat3* F) {
    int num_tets = body.tets.size();
    for (int tidx = 0; tidx < num_tets; tidx++) {
        auto& tet = body.tets[tidx];
        glm::rmat3 Dx = glm::rmat3(x[tet[0]] - x[tet[3]], x[tet[1]] - x[tet[3]], x[tet[2]] - x[tet[3]]) * body.B_m[tidx];
        F[tidx] = Dx + u[tidx];
    }
}

template <class Constraint>
void projective_dynamics_volume_constraint_local_solve(
        const SoftBody& body, const Constraint* constraints, uint32_t num_constraints,
        const glmx::SVD_mats<real>* F_svd, OUT glm::tmat3x3<real>* p) {

    for (int cidx = 0; cidx < num_constraints; cidx++) {
        auto& c = constraints[cidx];
        auto& svd = F_svd[c.tet_id];
        glm::rvec3 sigma = projection_eigvec(svd.Sigma, c);
        p[c.tet_id] = glmx::svd_mult(svd.U, sigma, svd.V);
    }
}

#define X(CTYPE, CFIELD) \
template void projective_dynamics_volume_constraint_local_solve( \
        const SoftBody&, const CTYPE*, uint32_t, \
        const glmx::SVD_mats<real>*, OUT glm::tmat3x3<real>*);
PD_VOLUME_CONSTRAINTS
#undef X

template <class Constraint>
void projective_dynamics_volume_constraint_update_b(
        const SoftBody& body, const Constraint* constraints, uint32_t num_constraints, real dt,
        const glm::tmat3x3<real>* p,
        INOUT real* b) {

    for (int cidx = 0; cidx < num_constraints; cidx++) {
        auto& c = constraints[cidx];
        glm::ivec4 tet = body.tets[c.tet_id];
        auto& D_i = body.D[c.tet_id];
        real k_s = c.k * body.W[c.tet_id];
        for (int j = 0; j < 4; j++) {
            glm::tvec3<real> db = k_s * (p[c.tet_id] * D_i[j]);
            b[3*tet[j]+0] += db[0];
            b[3*tet[j]+1] += db[1];
            b[3*tet[j]+2] += db[2];
        }
    }
}

#define X(CTYPE, CFIELD) \
template void projective_dynamics_volume_constraint_update_b( \
        const SoftBody& body, const CTYPE* constraints, uint32_t num_constraints, real dt, \
        const glm::tmat3x3<real>* F, \
        INOUT real* b);
PD_VOLUME_CONSTRAINTS
#undef X

void projective_dynamics_positional_constraint_update_b(
        const SoftBody& body, const PositionalConstraint* constraints, uint32_t num_constraints, real dt,
        INOUT real* b) {
    for (int cidx = 0; cidx < num_constraints; cidx++) {
        auto& c = constraints[cidx];
        b[3*c.vert_id+0] += c.k * c.target_pos.x;
        b[3*c.vert_id+1] += c.k * c.target_pos.y;
        b[3*c.vert_id+2] += c.k * c.target_pos.z;
    }
}

void projective_dynamics_collision_constraint_update_b(
        const SoftBody& body, const CollisionConstraint* constraints, uint32_t num_constraints, real dt,
        const glm::rvec3* x, INOUT real* b) {
    for (int cidx = 0; cidx < num_constraints; cidx++) {
        auto& c = constraints[cidx];
        real k_s = c.k * dt * dt;
        if (glm::dot(c.normal, x[c.vert_id] - c.target_pos) < 0) {
            auto dotp = glm::dot(c.normal, c.target_pos);
            b[3*c.vert_id+0] += k_s * c.normal.x * dotp;
            b[3*c.vert_id+1] += k_s * c.normal.y * dotp;
            b[3*c.vert_id+2] += k_s * c.normal.z * dotp;
        }
        else {
            auto dotp = glm::dot(c.normal, x[c.vert_id]);
            b[3*c.vert_id+0] += k_s * c.normal.x * dotp;
            b[3*c.vert_id+1] += k_s * c.normal.y * dotp;
            b[3*c.vert_id+2] += k_s * c.normal.z * dotp;
        }
    }
}

template <class Constraint>
void admm_volume_constraint_local_solve(
        const SoftBody& body, const Constraint* constraints, uint32_t num_constraints,
        const glm::rmat3* F, const glmx::SVD_mats<real>* F_svd,
        OUT glm::rmat3* z, OUT glm::rmat3* u) {
    for (int cidx = 0; cidx < num_constraints; cidx++) {
        auto& c = constraints[cidx];
        auto& svd = F_svd[c.tet_id];
        glm::rvec3 sigma = proximal_eigvec(svd.Sigma, body.W[c.tet_id], c);
        z[c.tet_id] = glmx::svd_mult(svd.U, sigma, svd.V);
        u[c.tet_id] = F[c.tet_id] - z[c.tet_id];
    }
}

#define X(CTYPE, CFIELD) \
template void admm_volume_constraint_local_solve( \
        const SoftBody& body, const CTYPE* constraints, uint32_t num_constraints, \
        const glm::rmat3* F, const glmx::SVD_mats<real>* F_svd, \
        OUT glm::rmat3* z, OUT glm::rmat3* u);
ADMM_VOLUME_CONSTRAINTS
#undef X

template <class Constraint>
void admm_volume_constraint_update_b(
        const SoftBody& body, const Constraint* constraints, uint32_t num_constraints,
        real dt, const glm::rmat3* z, const glm::rmat3* u,
        INOUT real* b) {

    for (int cidx = 0; cidx < num_constraints; cidx++) {
        auto& c = constraints[cidx];
        glm::ivec4 tet = body.tets[c.tet_id];
        auto p = z[c.tet_id] - u[c.tet_id];
        auto& D_i = body.D[c.tet_id];
        real k_s = c.k * body.W[c.tet_id];
        for (int j = 0; j < 4; j++) {
            glm::tvec3<real> db = k_s * (p * D_i[j]);
            b[3*tet[j]+0] += db[0];
            b[3*tet[j]+1] += db[1];
            b[3*tet[j]+2] += db[2];
        }
    }
}

#define X(CTYPE, CFIELD) \
template void admm_volume_constraint_update_b( \
        const SoftBody& body, const CTYPE* constraints, uint32_t num_constraints, \
        real dt, const glm::rmat3* z, const glm::rmat3* u, INOUT real* b);
ADMM_VOLUME_CONSTRAINTS
#undef X

void admm_positional_constraint_update_b(
        const SoftBody& body, const PositionalConstraint* constraints, uint32_t num_constraints, real dt,
        INOUT real* b) {
    for (int cidx = 0; cidx < num_constraints; cidx++) {
        auto& c = constraints[cidx];
        b[3*c.vert_id+0] += c.k * c.target_pos.x;
        b[3*c.vert_id+1] += c.k * c.target_pos.y;
        b[3*c.vert_id+2] += c.k * c.target_pos.z;
    }
}
template <class Constraint>
void admm_volume_constraint_update_residuals(
        const SoftBody& body, const Constraint* constraints, uint32_t num_constraints,
        const glm::tmat3x3<real>* z_prev, const glm::tmat3x3<real>* z_next, const glm::tvec3<real>* x,
        INOUT real& primal_res_sq, INOUT real& dual_res_sq) {
    for (int cidx = 0; cidx < num_constraints; cidx++) {
        auto& c = constraints[cidx];
        glm::ivec4 tet = body.tets[c.tet_id];
        auto& D_i = body.D[c.tet_id];
        auto D_x = glm::rmat3(x[tet[0]] - x[tet[3]], x[tet[1]] - x[tet[3]], x[tet[2]] - x[tet[3]]) * body.B_m[c.tet_id];
        primal_res_sq += c.k * glmx::length2(D_x - z_next[c.tet_id]);
        dual_res_sq += c.k * glmx::length2(z_next[c.tet_id] - z_prev[c.tet_id]);
    }
}

#define X(CTYPE, CFIELD) \
template void admm_volume_constraint_update_residuals( \
        const SoftBody& body, const CTYPE* constraints, uint32_t num_constraints, \
        const glm::tmat3x3<real>* z_prev, const glm::tmat3x3<real>* z_next, const glm::tvec3<real>* x, \
        INOUT real& primal_res_sq, INOUT real& dual_res_sq);
ADMM_VOLUME_CONSTRAINTS
#undef X

void projective_dynamics(const SoftBody& body,
                         const PDConstraints& constraints, real dt, int num_iters, const real* f,
                         INOUT real* pos, INOUT real* vel) {
    Map<VectorXr> x(pos, 3*body.verts.size());
    Map<VectorXr> v(vel, 3*body.verts.size());
    Map<const VectorXr> f_ext(f, 3*body.verts.size());
    VectorXr x_orig = x;
    VectorXr x_tilde = x + dt*v + body.M_LDLt.solve(f_ext);

    std::vector<glm::tmat3x3<real>> F(body.tets.size());
    std::vector<glmx::SVD_mats<real>> F_svd(body.tets.size());
    std::vector<glm::tmat3x3<real>> p(body.tets.size());

    for (int iter = 0; iter < num_iters; iter++) {
        soft_body_calc_deformation_field(body, (glm::rvec3*) x.data(), OUT F.data());
        glmx::fastsvd(F.data(), body.tets.size(), OUT F_svd.data());

        // Local solve
#define X(CTYPE, CFIELD) \
        projective_dynamics_volume_constraint_local_solve( \
                body, constraints.CFIELD.data(), constraints.CFIELD.size(), F_svd.data(), OUT p.data());
        PD_VOLUME_CONSTRAINTS
#undef X

        // Global solve
        VectorXr b = body.M * x_tilde;
#define X(CTYPE, CFIELD) \
        projective_dynamics_volume_constraint_update_b( \
                body, constraints.CFIELD.data(), constraints.CFIELD.size(), dt, p.data(), INOUT b.data());
        PD_VOLUME_CONSTRAINTS
#undef X
        projective_dynamics_positional_constraint_update_b(
                body, constraints.positional.data(), constraints.positional.size(), dt, INOUT b.data());

        projective_dynamics_collision_constraint_update_b(
                body, constraints.soft_rigid_collision.data(), constraints.soft_rigid_collision.size(), dt,
                (glm::rvec3*) x.data(), INOUT b.data());

        x = body.A_LDLt.solve(b);
    }
    v = (x - x_orig) / dt;
}

void projective_dynamics_quasistatic(const SoftBody& body,
                                     const PDConstraints& constraints, int num_iters, const real* f,
                                     INOUT real* pos) {
    Map<VectorXr> x(pos, 3*body.verts.size());
    Map<const VectorXr> f_ext(f, 3*body.verts.size());

    std::vector<glm::tmat3x3<real>> F(body.tets.size());
    std::vector<glmx::SVD_mats<real>> F_svd(body.tets.size());
    std::vector<glm::tmat3x3<real>> p(body.tets.size());

    for (int iter = 0; iter < num_iters; iter++) {
        soft_body_calc_deformation_field(body, (glm::rvec3*) x.data(), OUT F.data());
        glmx::fastsvd(F.data(), body.tets.size(), OUT F_svd.data());

        // Global solve
#define X(CTYPE, CFIELD) \
        projective_dynamics_volume_constraint_local_solve( \
                body, constraints.CFIELD.data(), constraints.CFIELD.size(), F_svd.data(), OUT p.data());
        PD_VOLUME_CONSTRAINTS
#undef X

        // Local solve
        VectorXr b = f_ext;
#define X(CTYPE, CFIELD) \
        projective_dynamics_volume_constraint_update_b( \
                body, constraints.CFIELD.data(), constraints.CFIELD.size(), 1.0, p.data(), INOUT b.data());
        PD_VOLUME_CONSTRAINTS
#undef X
        projective_dynamics_positional_constraint_update_b(
                body, constraints.positional.data(), constraints.positional.size(), 1.0, INOUT b.data());
        projective_dynamics_collision_constraint_update_b(
                body, constraints.soft_rigid_collision.data(), constraints.soft_rigid_collision.size(), 1.0,
                (glm::rvec3*) x.data(), INOUT b.data());

        x = body.A_LDLt.solve(b);
    }
}

void admm_dynamics(const SoftBody& body,
                   const ADMMConstraints& constraints, real dt, int num_iters, const real* f,
                   INOUT real* pos, INOUT real* vel) {

    int num_vertices = body.verts.size();
    int num_tets = body.tets.size();

    Map<VectorXr> x(pos, 3*num_vertices);
    Map<VectorXr> v(vel, 3*num_vertices);
    Map<const VectorXr> f_ext(f, 3*num_vertices);
    VectorXr x_orig = x;
    VectorXr x_tilde = x + dt*v + body.M_LDLt.solve(f_ext);
    x = x_tilde;

    std::vector<glm::tmat3x3<real>> u(num_tets, glm::tmat3x3<real>(0.0));
    std::vector<glm::tmat3x3<real>> z(num_tets, glm::tmat3x3<real>(0.0));
    std::vector<glm::tmat3x3<real>> z_prev(num_tets);
    std::vector<glm::tmat3x3<real>> F(num_tets);
    std::vector<glmx::SVD_mats<real>> F_svd(num_tets);

    fmt::print("\nStarting ADMM loop\n");
    for (int iter = 0; iter < num_iters; iter++) {
        z_prev = z;

        for (int tidx = 0; tidx < num_tets; tidx++) {
            auto& tet = body.tets[tidx];
            glm::rvec3* X = (glm::rvec3*) x.data();
            glm::rmat3 Dx = glm::rmat3(X[tet[0]] - X[tet[3]], X[tet[1]] - X[tet[3]], X[tet[2]] - X[tet[3]]) * body.B_m[tidx];
            F[tidx] = Dx + u[tidx];
        }
        glmx::fastsvd(F.data(), num_tets, F_svd.data());

        // Local solve
#define X(CTYPE, CFIELD) \
        admm_volume_constraint_local_solve(body, constraints.CFIELD.data(), constraints.CFIELD.size(), \
            F.data(), F_svd.data(), OUT z.data(), OUT u.data());
        ADMM_VOLUME_CONSTRAINTS
#undef X

        // Global solve
        VectorXr b = body.M * x_tilde;

#define X(CTYPE, CFIELD) \
        admm_volume_constraint_update_b(body, constraints.CFIELD.data(), constraints.CFIELD.size(), dt, z.data(), u.data(), \
            OUT b.data());
        ADMM_VOLUME_CONSTRAINTS
#undef X
        projective_dynamics_positional_constraint_update_b(body, constraints.positional.data(),
                                                           constraints.positional.size(), dt, OUT b.data());

        x.noalias() = body.A_LDLt.solve(b);

        real primal_res_sq = 0, dual_res_sq = 0;
#define X(CTYPE, CFIELD) \
        admm_volume_constraint_update_residuals(body, constraints.CFIELD.data(), constraints.CFIELD.size(), \
            z_prev.data(), z.data(), (glm::rvec3*)x.data(), INOUT primal_res_sq, INOUT dual_res_sq);
        ADMM_VOLUME_CONSTRAINTS
#undef X

        fmt::print("primal_res = {}, dual_res = {}\n", sqrt(primal_res_sq), sqrt(dual_res_sq));
    }
    v = (x - x_orig) / dt;
}

real quasinewton_dynamics_objective_fn(
        const SoftBody& body, const ADMMConstraints& constraints, real dt,
        const VectorXr& x, const VectorXr& x_tilde, OUT glm::rmat3* F, OUT glmx::SVD_mats<real>* F_svd) {
    soft_body_calc_deformation_field(body, (glm::rvec3*) x.data(), OUT F);
    glmx::fastsvd(F, body.tets.size(), OUT F_svd);
    VectorXr dx = x - x_tilde;
    real E = real(0.5) * dx.dot(body.M * dx);
    real dt_sq = dt * dt;
#define X(CTYPE, CFIELD) \
    for (int cidx = 0; cidx < constraints.CFIELD.size(); cidx++) { \
        auto& c = constraints.CFIELD[cidx]; \
        E += dt_sq * c.k * body.W[c.tet_id] * energy_eigvec(F_svd[c.tet_id].Sigma, c); \
    }
    ADMM_VOLUME_CONSTRAINTS
#undef X
    for (int cidx = 0; cidx < constraints.positional.size(); cidx++) {
        auto& c = constraints.positional[cidx];
        E += dt_sq * c.k * glm::length2(x[c.vert_id] - c.target_pos);
    }
    return E;
}


template <class Constraint>
void quasinewton_dynamics_volume_constraint_energy_gradient(
        const SoftBody& body, const Constraint* constraints, uint32_t num_constraints, real dt,
        const glm::rmat3* F, const glmx::SVD_mats<real>* F_svd, OUT glm::rvec3* E_grad) {

    for (int cidx = 0; cidx < num_constraints; cidx++) {
        auto& c = constraints[cidx];
        glm::ivec4 tet = body.tets[c.tet_id];
        auto& D_i = body.D[c.tet_id];
        glm::rmat3 pk1_tensor = pk1(F[c.tet_id], F_svd[c.tet_id], c);
        real k_s = c.k * body.W[c.tet_id];
        for (int j = 0; j < 4; j++) {
            E_grad[tet[j]] += k_s * (pk1_tensor * D_i[j]);
        }
    }
}

void quasinewton_dynamics_positional_constraint_energy_gradient(
        const SoftBody& body, const PositionalConstraint* constraints, uint32_t num_constraints, real dt,
        const glm::rvec3* x, OUT glm::rvec3* E_grad) {
    real dt_sq = dt * dt;
    for (int cidx = 0; cidx < num_constraints; cidx++) {
        auto& c = constraints[cidx];
        E_grad[c.vert_id] += c.k * (x[c.vert_id] - c.target_pos);
    }
}

void quasinewton_dynamics_objective_fn_grad(
        const SoftBody& body, const ADMMConstraints& constraints, real dt,
        const glm::rmat3* F, const glmx::SVD_mats<real>* F_svd, const VectorXr& x, const VectorXr& x_tilde, OUT VectorXr& g_x_grad) {
    g_x_grad = body.M * (x - x_tilde);
    real dt_sq = dt * dt;
#define X(CTYPE, CFIELD) \
    quasinewton_dynamics_volume_constraint_energy_gradient( \
        body, constraints.CFIELD.data(), constraints.CFIELD.size(), dt, F, F_svd, OUT (glm::rvec3*)g_x_grad.data());
    ADMM_VOLUME_CONSTRAINTS
#undef X
    quasinewton_dynamics_positional_constraint_energy_gradient(
            body, constraints.positional.data(), constraints.positional.size(), dt, (glm::rvec3*)x.data(), OUT (glm::rvec3*)g_x_grad.data());

}

struct SoftBodyQuasiNewtonHistory {
    std::deque<VectorXr> x;
    std::deque<VectorXr> s, t;
    std::deque<real> rho, zeta;
    int max_hist_count = 5;

    explicit SoftBodyQuasiNewtonHistory(int max_hist_count) : max_hist_count(max_hist_count) {}

    void calc_descent_dir(const SoftBody& body,
                          const VectorXr& new_x, const VectorXr& new_grad,
                          OUT VectorXr& r) {
        VectorXr q = -new_grad;

        if (x.empty()) {
            r = body.A_LDLt.solve(q);
            x.push_front(new_x);
            return;
        }

        VectorXr s0 = new_x - x[0];
        VectorXr t0 = body.A * s0;
        real rho0 = s0.dot(t0);
        real zeta0 = s0.dot(q) / rho0;

        x.push_front(new_x);
        s.push_front(s0);
        t.push_front(t0);
        rho.push_front(rho0);
        zeta.push_front(zeta0);
        if (s.size() > max_hist_count) {
            x.pop_back();
            s.pop_back();
            t.pop_back();
            rho.pop_back();
            zeta.pop_back();
        }

        // TODO: cache this summed value!
        for (int i = 0; i < t.size(); i++) {
            q -= zeta[i] * t[i];
        }

        r = body.A_LDLt.solve(q);
        // TODO: cache this summed value!
        for (int i = t.size()-1; i >= 0; i--) {
            real eta = t[i].dot(r) / rho[i];
            r += s[i] * (zeta[i] - eta);
        }
    }
};

void quasinewton_dynamics(const SoftBody& body,
                          const ADMMConstraints& constraints, real dt, int num_iters, const real* f,
                          INOUT real* pos, INOUT real* vel) {
    Map<VectorXr> x(pos, 3*body.verts.size());
    Map<VectorXr> v(vel, 3*body.verts.size());
    Map<const VectorXr> f_ext(f, 3*body.verts.size());
    VectorXr x_orig = x;
    VectorXr x_tilde = x + dt*v + body.M_LDLt.solve(f_ext);

    std::vector<glm::tmat3x3<real>> F(body.tets.size());
    std::vector<glmx::SVD_mats<real>> F_svd(body.tets.size());
    std::vector<glm::tmat3x3<real>> p(body.tets.size());

    // Begin Quasi-Newton solver
    SoftBodyQuasiNewtonHistory hist(5);
    const real gamma = 0.3;
    const int max_bt_iters = 10;
    x = x_tilde;
    real g_x0 = quasinewton_dynamics_objective_fn(
            body, constraints, dt, x, x_tilde, OUT F.data(), OUT F_svd.data());
    printf("g_x0 = %f\n", g_x0);
    VectorXr g_x0_grad(x.size());
    VectorXr d_x0(x.size());
    VectorXr x_cur;
    for (int k = 1; k <= num_iters; k++) {
        quasinewton_dynamics_objective_fn_grad(body, constraints, dt, F.data(), F_svd.data(),
                                               x, x_tilde, OUT g_x0_grad);
        // hist.calc_descent_dir(body, precalc, x, g_x0_grad, OUT d_x0);
        d_x0 = -body.A_LDLt.solve(g_x0_grad);
        // std::cout << d_x0.transpose() << std::endl;
        real alpha = real(1.0);
        real g_x;
        x_cur = x;
        for (int iter = 0; iter < max_bt_iters; iter++) {
            x = x_cur + alpha * d_x0;
            g_x = quasinewton_dynamics_objective_fn(
                    body, constraints, dt, x, x_tilde, OUT F.data(), OUT F_svd.data());
            real g_x_threshold = g_x0 + gamma * alpha * g_x0_grad.dot(d_x0);
            printf("g_x = %f, g_x_threshold = %f\n", g_x, g_x_threshold);
            if (g_x < g_x_threshold) break;
            alpha = alpha / 2;
        }
    }
    v = (x - x_orig) / dt;
}

}
