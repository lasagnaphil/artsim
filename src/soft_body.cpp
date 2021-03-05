//
// Created by lasagnaphil on 21. 2. 8..
//

#include "artsim/soft_body.h"
#include "artsim/math/svd.h"
#include "artsim/math/fastsvd.h"
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/IterativeLinearSolvers>
#include <unordered_map>
#include <glm/gtx/hash.hpp>
#include <glm/gtx/string_cast.hpp>

using namespace Eigen;
using MatrixXr = Matrix<artsim::real, Dynamic, Dynamic>;
using VectorXr = Matrix<artsim::real, Dynamic, 1>;

namespace artsim {

std::pair<glm::ivec3, bool> reorder_tri_indices(glm::ivec3 tri) {
    bool flipped = false;
    if (tri[1] > tri[2]) {
        flipped = !flipped;
        std::swap(tri[1], tri[2]);
    }
    if (tri[0] > tri[1]) {
        flipped = !flipped;
        std::swap(tri[0], tri[1]);
    }
    if (tri[1] > tri[2]) {
        flipped = !flipped;
        std::swap(tri[1], tri[2]);
    }
    return {tri, flipped};
}

void gen_surface_triangles_from_tet_mesh(const std::vector<glm::ivec4>& tetrahedrons,
                                         OUT std::vector<glm::ivec3>& triangles) {
#if 0
    for (auto& tet : tetrahedrons) {
        triangles.push_back({tet[0], tet[2], tet[1]});
        triangles.push_back({tet[0], tet[1], tet[3]});
        triangles.push_back({tet[0], tet[3], tet[2]});
        triangles.push_back({tet[1], tet[2], tet[3]});
    }
#else
    std::unordered_map<glm::ivec3, std::pair<int, bool>> tri_overlaps;
    auto insert_triangle = [&](glm::ivec3 tri) {
        auto [tri_p, flipped] = reorder_tri_indices(tri);
        auto it = tri_overlaps.find(tri_p);
        if (it == tri_overlaps.end()) {
            tri_overlaps.insert({tri_p, {1, flipped}});
        }
        else {
            it->second.first++;
        }
    };
    for (auto& tet : tetrahedrons) {
        insert_triangle({tet[0], tet[2], tet[1]});
        insert_triangle({tet[0], tet[1], tet[3]});
        insert_triangle({tet[0], tet[3], tet[2]});
        insert_triangle({tet[1], tet[2], tet[3]});
    }
    for (auto [tri, p] : tri_overlaps) {
        auto [count, flipped] = p;
        if (count == 1) {
            if (flipped) {
                glm::ivec3 tri_p(tri[1], tri[0], tri[2]);
                triangles.push_back(tri_p);
            }
            else {
                triangles.push_back(tri);
            }
        }
    }
#endif
}

void SoftBodyData::load(const OBJFile& obj, const SoftBodyProperties& props) {
    this->props = props;
    vertices = obj.vertices;
    tetrahedrons = obj.tetrahedrons;
    gen_surface_triangles_from_tet_mesh(tetrahedrons, OUT triangles);
}

void SoftBodyData::load(const PyMesh::MshLoader& msh, const SoftBodyProperties& props) {
    this->props = props;
    auto& nodes = msh.get_nodes();
    auto& elems = msh.get_elements();
    std::cout << "nodes =" << nodes.size() << ", elems=" << elems.size() << std::endl;
    int num_nodes = nodes.rows() / 3;
    int num_elems = elems.rows() / 4;
    vertices.resize(num_nodes);
    for (int i = 0; i < num_nodes; i++) {
        vertices[i] = {nodes[3*i+0], nodes[3*i+1], nodes[3*i+2]};
    }
    tetrahedrons.resize(num_elems);
    for (int i = 0; i < num_elems; i++) {
        tetrahedrons[i] = {elems[4*i+0], elems[4*i+1], elems[4*i+2], elems[4*i+3]};
    }
    gen_surface_triangles_from_tet_mesh(tetrahedrons, OUT triangles);
}

void tetrahedral_mesh_mass_matrix(int num_vertices, real density,
                           const glm::ivec4* tets, int num_tets,
                           const real* tet_volumes,
                           OUT SparseMatrix<real>& M) {

    M.resize(3*num_vertices, 3*num_vertices);
    std::unordered_map<glm::ivec2, real> M_triplets_map;
    for (int i = 0; i < num_tets; i++) {
        real m = density * tet_volumes[i] / 20.0;
        glm::ivec4 tet = tets[i];
        for (int j = 0; j < 4; j++) {
            for (int k = 0; k < 4; k++) {
                for (int l = 0; l < 3; l++) {
                    auto idx = glm::ivec2(3*tet[j]+l, 3*tet[k]+l);
                    auto it = M_triplets_map.find(idx);
                    if (it != M_triplets_map.end()) {
                        it->second += j == k? 2.0*m : m;
                    }
                    else {
                        M_triplets_map.insert({idx, j == k? 2.0*m : m});
                    }
                }
            }
        }
    }
    std::vector<Triplet<real>> M_triplets;
    M_triplets.reserve(M_triplets_map.size());
    for (auto& [k, v] : M_triplets_map) {
        M_triplets.emplace_back(k[0], k[1], v);
    }
    M.setFromTriplets(M_triplets.begin(), M_triplets.end());
}

void precomputation_essentials(SoftBodyData& body) {
    body.B_m.resize(body.tetrahedrons.size());
    body.W.resize(body.tetrahedrons.size());
    body.D.resize(body.tetrahedrons.size());
    for (int i = 0; i < body.tetrahedrons.size(); i++) {
        auto& V = body.vertices;
        glm::ivec4& tet = body.tetrahedrons[i];
        glm::tmat3x3<real> D_m(V[tet[0]] - V[tet[3]], V[tet[1]] - V[tet[3]], V[tet[2]] - V[tet[3]]);
        body.W[i] = glm::determinant(D_m) / 6.0;
        if (body.W[i] < 0) {
            body.W[i] = -body.W[i];
            std::swap(tet[2], tet[3]);
            D_m = glm::tmat3x3<real>(V[tet[0]] - V[tet[3]], V[tet[1]] - V[tet[3]], V[tet[2]] - V[tet[3]]);
        }
        body.B_m[i] = glm::inverse(D_m);
        glm::tmat3x3<real> D_i = glm::transpose(body.B_m[i]);
        body.D[i][0] = D_i[0];
        body.D[i][1] = D_i[1];
        body.D[i][2] = D_i[2];
        body.D[i][3] = -D_i[0] - D_i[1] - D_i[2];
    }

    tetrahedral_mesh_mass_matrix(body.vertices.size(), body.props.density,
                                 body.tetrahedrons.data(), body.tetrahedrons.size(), body.W.data(),
                                 OUT body.M);
    body.M_LDLt.analyzePattern(body.M);
    body.M_LDLt.factorize(body.M);
}

template <>
void update_system_matrix(SoftBodyData& body, const PDConstraints& constraints, real dt, OUT SparseMatrix<real>& A) {
    A = body.M;
#define X(CTYPE, CFIELD) \
    for (const auto& c : constraints.CFIELD) { \
        glm::ivec4 tet = body.tetrahedrons[c.tet_id]; \
        for (int j = 0; j < 4; j++) { \
            for (int k = 0; k < 4; k++) { \
                real dA = dt * dt * c.k * body.W[c.tet_id] * glm::dot(body.D[c.tet_id][j], body.D[c.tet_id][k]); \
                A.coeffRef(3*tet[j]+0, 3*tet[k]+0) += dA; \
                A.coeffRef(3*tet[j]+1, 3*tet[k]+1) += dA; \
                A.coeffRef(3*tet[j]+2, 3*tet[k]+2) += dA; \
            } \
        } \
    }
    PD_VOLUME_CONSTRAINTS
#undef X
}

template <>
void update_system_matrix(SoftBodyData& body, const ADMMConstraints& constraints, real dt, OUT SparseMatrix<real>& A) {
    A = body.M;
#define X(CTYPE, CFIELD) \
    for (const auto& c : constraints.CFIELD) { \
        glm::ivec4 tet = body.tetrahedrons[c.tet_id]; \
        for (int j = 0; j < 4; j++) { \
            for (int k = 0; k < 4; k++) { \
                real dA = dt * dt * c.k * body.W[c.tet_id] * glm::dot(body.D[c.tet_id][j], body.D[c.tet_id][k]); \
                A.coeffRef(3*tet[j]+0, 3*tet[k]+0) += dA; \
                A.coeffRef(3*tet[j]+1, 3*tet[k]+1) += dA; \
                A.coeffRef(3*tet[j]+2, 3*tet[k]+2) += dA; \
            } \
        } \
    }
    ADMM_VOLUME_CONSTRAINTS
#undef X
}

template <class Constraints>
void soft_body_precomputation(SoftBodyData& body, const Constraints& constraints, real dt) {
    precomputation_essentials(body);

    update_system_matrix(body, constraints, dt, OUT body.A);

    body.A_LDLt.analyzePattern(body.A);
    body.A_LDLt.factorize(body.A);
}

template void soft_body_precomputation<PDConstraints>(SoftBodyData& body, const PDConstraints& constraints, real dt);
template void soft_body_precomputation<ADMMConstraints>(SoftBodyData& body, const ADMMConstraints& constraints, real dt);

glm::tvec3<real> calc_S_star(glm::tvec3<real> S, real sigma_min, real sigma_max) {
    real sigma = S[0]*S[1]*S[2];
    if (sigma < sigma_min) sigma = sigma_min;
    if (sigma > sigma_max) sigma = sigma_max;
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

glm::tmat3x3<real> projection(const glm::tmat3x3<real>& F, const LinearStrainEnergyConstraint& c) {
    fprintf(stderr, "Unimplemented!");
    exit(EXIT_FAILURE);
}

glm::tmat3x3<real> projection(const glm::tmat3x3<real>& F, const VolumePreservationEnergyConstraint& c) {
    auto F_svd = glmx::svd(F);
    F_svd.Sigma = calc_S_star(F_svd.Sigma, c.sigma_min, c.sigma_max);
    auto Sigma = glm::tmat3x3<real>(F_svd.Sigma[0], 0, 0, 0, F_svd.Sigma[1], 0, 0, 0, F_svd.Sigma[2]);
    return F_svd.U * Sigma * glm::transpose(F_svd.V);
}

glm::tvec3<real> proximal_eigvec(glm::tvec3<real> sigma, const CorotationalEnergyConstraint& c) {
    real A_diag = 2*c.mu + c.lambda + c.k;
    glmx::tsmat3x3<real> A(A_diag, A_diag, A_diag, c.lambda, c.lambda, c.lambda);
    glm::tvec3<real> b(2*c.mu + 3*c.lambda + c.k*sigma.x,
                       2*c.mu + 3*c.lambda + c.k*sigma.y,
                       2*c.mu + 3*c.lambda + c.k*sigma.z);
    return inverse(A) * b;
}

glm::tmat3x3<real> proximal(const glm::tmat3x3<real>& F, const CorotationalEnergyConstraint& c) {
    auto F_svd = glmx::svd(F);
    F_svd.Sigma = proximal_eigvec(F_svd.Sigma, c);
    glm::tmat3x3<real> Sigma(F_svd.Sigma.x, 0, 0, 0, F_svd.Sigma.y, 0, 0, 0, F_svd.Sigma.z);
    return F_svd.U * Sigma * glm::transpose(F_svd.V);
}

glm::tvec3<real> proximal_eigvec(glm::tvec3<real> sigma, const NeoHookeanEnergyConstraint& c) {
    auto S = sigma;
    for (int i = 0; i < 5; i++) {
        real J = log(S[0]*S[1]*S[2]);
        glm::tvec3<real> grad;
        grad[0] = c.mu*(S[0] - 1.0/S[0]) + c.lambda/S[0] * J + c.k*(S[0] - sigma[0]);
        grad[1] = c.mu*(S[1] - 1.0/S[1]) + c.lambda/S[1] * J + c.k*(S[1] - sigma[1]);
        grad[2] = c.mu*(S[2] - 1.0/S[2]) + c.lambda/S[2] * J + c.k*(S[2] - sigma[2]);
        glmx::tsmat3x3<real> H;
        H.xx = c.mu + (c.mu + c.lambda)/(S[0]*S[0]) - c.lambda/(S[0]*S[0]) * J + c.k;
        H.yy = c.mu + (c.mu + c.lambda)/(S[1]*S[1]) - c.lambda/(S[1]*S[1]) * J + c.k;
        H.zz = c.mu + (c.mu + c.lambda)/(S[2]*S[2]) - c.lambda/(S[2]*S[2]) * J + c.k;
        H.yz = c.lambda / (S[1]*S[2]);
        H.zx = c.lambda / (S[2]*S[0]);
        H.xy = c.lambda / (S[0]*S[1]);
        /*
        std::cout << glm::to_string(S) << std::endl;
        if (glm::isnan(H.xx) || glm::isnan(H.yy) || glm::isnan(H.zz)) {
            std::cout << "Nan detected!" << std::endl;
        }
        if (glm::epsilonEqual(glmx::determinant(H), 0., 1e-8)) {
            std::cout << "Singular matrix!" << std::endl;
        }
         */
        S -= glmx::inverse(H) * grad;
        S = glm::max(S, glm::rvec3(0)); // Prevent volume from becoming negative
    }
    // std::cout << "opt finished" << std::endl;
    return S;
}

glm::tmat3x3<real> proximal(const glm::tmat3x3<real>& F, const NeoHookeanEnergyConstraint& c) {
    auto F_svd = glmx::svd(F);
    F_svd.Sigma = proximal_eigvec(F_svd.Sigma, c);
    glm::tmat3x3<real> Sigma(F_svd.Sigma.x, 0, 0, 0, F_svd.Sigma.y, 0, 0, 0, F_svd.Sigma.z);
    return F_svd.U * Sigma * glm::transpose(F_svd.V);
}


template <class Constraint>
void projective_dynamics_volume_constraint_local_solve(
        const SoftBodyData& body, const Constraint* constraints, uint32_t num_constraints,
        const glm::tvec3<real>* V,
        OUT glm::tmat3x3<real>* p) {

// #pragma omp parallel for schedule(static)
    for (int cidx = 0; cidx < num_constraints; cidx++) {
        auto& c = constraints[cidx];
        glm::ivec4 tet = body.tetrahedrons[c.tet_id];
        glm::tmat3x3<real> D_s(V[tet[0]] - V[tet[3]], V[tet[1]] - V[tet[3]], V[tet[2]] - V[tet[3]]);
        glm::tmat3x3<real> F = D_s * body.B_m[c.tet_id];
        p[c.tet_id] = projection(F, c);
    }
}

#define X(CTYPE, CFIELD) \
template void projective_dynamics_volume_constraint_local_solve( \
        const SoftBodyData&, const CTYPE*, uint32_t, const glm::tvec3<real>*, OUT glm::tmat3x3<real>*);
PD_VOLUME_CONSTRAINTS
#undef X

template <class Constraint>
void admm_volume_constraint_local_solve(
        const SoftBodyData& body, const Constraint* constraints, uint32_t num_constraints,
        const glm::tvec3<real>* x,
        OUT glm::tmat3x3<real>* z, OUT glm::tmat3x3<real>* u,
        OUT glm::tmat3x3<real>* F, OUT glmx::SVD_mats<real>* F_svd) {

// #pragma omp parallel for schedule(static)
    for (int cidx = 0; cidx < num_constraints; cidx++) {
        auto& c = constraints[cidx];
        glm::ivec4 tet = body.tetrahedrons[c.tet_id];
        auto D_x = glm::rmat3(x[tet[0]] - x[tet[3]], x[tet[1]] - x[tet[3]], x[tet[2]] - x[tet[3]]) * body.B_m[c.tet_id];
        F[c.tet_id] = D_x + u[c.tet_id];
    }

    glmx::fastsvd(F, num_constraints, F_svd);

// #pragma omp parallel for schedule(static)
    for (int cidx = 0; cidx < num_constraints; cidx++) {
        auto& c = constraints[cidx];
        F_svd[c.tet_id].Sigma = proximal_eigvec(F_svd[c.tet_id].Sigma, c);
        z[c.tet_id] = F_svd[c.tet_id].recover_matrix();
        u[c.tet_id] = F[c.tet_id] - z[c.tet_id];
    }
}

#define X(CTYPE, CFIELD) \
template void admm_volume_constraint_local_solve_fast( \
        const SoftBodyData&, const CTYPE*, uint32_t, \
        const glm::tvec3<real>*, \
        OUT glm::tmat3x3<real>*, OUT glm::tmat3x3<real>*, \
        OUT glm::tmat3x3<real>* F, OUT glmx::SVD_mats<real>* F_svd); \
ADMM_VOLUME_CONSTRAINTS
#undef X

template <class Constraint>
void admm_volume_constraint_update_b(
        const SoftBodyData& body, const Constraint* constraints, uint32_t num_constraints,
        real dt, const glm::tmat3x3<real>* z, const glm::tmat3x3<real>* u, const glm::tvec3<real>* x0,
        INOUT real* b) {

    for (int cidx = 0; cidx < num_constraints; cidx++) {
        auto& c = constraints[cidx];
        glm::ivec4 tet = body.tetrahedrons[c.tet_id];
        auto p = z[c.tet_id] - u[c.tet_id];
        auto& D_i = body.D[c.tet_id];
        auto D_x0 = glm::rmat3(x0[tet[0]] - x0[tet[3]], x0[tet[1]] - x0[tet[3]], x0[tet[2]] - x0[tet[3]]) * body.B_m[c.tet_id];
        real k_s = dt * c.k * body.W[c.tet_id];
        for (int j = 0; j < 4; j++) {
            glm::tvec3<real> db = k_s * ((p - D_x0) * D_i[j]);
            b[3*tet[j]+0] += db[0];
            b[3*tet[j]+1] += db[1];
            b[3*tet[j]+2] += db[2];
        }
    }
}

#define X(CTYPE, CFIELD) \
template void admm_volume_constraint_update_b( \
        const SoftBodyData& body, const CTYPE* constraints, uint32_t num_constraints, \
        real dt, const glm::tmat3x3<real>* z, const glm::tmat3x3<real>* u, const glm::tvec3<real>* x0, INOUT real* b);
ADMM_VOLUME_CONSTRAINTS
#undef X

template <class Constraint>
void admm_volume_constraint_update_residuals(
        const SoftBodyData& body, const Constraint* constraints, uint32_t num_constraints,
        const glm::tmat3x3<real>* z_prev, const glm::tmat3x3<real>* z_next, const glm::tvec3<real>* x,
        INOUT real& primal_res_sq, INOUT real& dual_res_sq) {
    for (int cidx = 0; cidx < num_constraints; cidx++) {
        auto& c = constraints[cidx];
        glm::ivec4 tet = body.tetrahedrons[c.tet_id];
        auto& D_i = body.D[c.tet_id];
        auto D_x = glm::rmat3(x[tet[0]] - x[tet[3]], x[tet[1]] - x[tet[3]], x[tet[2]] - x[tet[3]]) * body.B_m[c.tet_id];
        primal_res_sq += c.k * glmx::length2(D_x - z_next[c.tet_id]);
        for (int j = 0; j < 4; j++) {
            dual_res_sq += c.k * c.k * glm::length2((z_next[c.tet_id] - z_prev[c.tet_id]) * D_i[j]);
        }
    }
}

#define X(CTYPE, CFIELD) \
template void admm_volume_constraint_update_residuals( \
        const SoftBodyData& body, const CTYPE* constraints, uint32_t num_constraints, \
        const glm::tmat3x3<real>* z_prev, const glm::tmat3x3<real>* z_next, const glm::tvec3<real>* x, \
        INOUT real& primal_res_sq, INOUT real& dual_res_sq);
ADMM_VOLUME_CONSTRAINTS
#undef X

void admm_dynamics(SoftBodyData& body, const ADMMConstraints& constraints, real dt, const real* f,
                   INOUT real* pos, INOUT real* vel) {

    Map<VectorXr> x(pos, 3*body.vertices.size());
    Map<VectorXr> v(vel, 3*body.vertices.size());
    Map<const VectorXr> f_ext(f, 3*body.vertices.size());
    VectorXr x_orig = x;
    VectorXr v_tilde = v + dt*body.M_LDLt.solve(f_ext);
    v.noalias() = v_tilde;
    x.noalias() = x_orig + dt*v;

    std::vector<glm::tmat3x3<real>> u(body.tetrahedrons.size(), glm::tmat3x3<real>(0.0));
    std::vector<glm::tmat3x3<real>> z(body.tetrahedrons.size(), glm::tmat3x3<real>(0.0));
    std::vector<glm::tmat3x3<real>> z_prev(body.tetrahedrons.size());
    std::vector<glm::tmat3x3<real>> F(body.tetrahedrons.size());
    std::vector<glmx::SVD_mats<real>> F_svd(body.tetrahedrons.size());

    std::cout << std::endl << "Starting ADMM loop" << std::endl;
    for (int iter = 0; iter < 10; iter++) {
        z_prev = z;

        // Local solve
#define X(CTYPE, CFIELD) \
        admm_volume_constraint_local_solve(body, constraints.CFIELD.data(), constraints.CFIELD.size(), \
            (glm::rvec3*) x.data(), OUT z.data(), OUT u.data(), OUT F.data(), OUT F_svd.data());
        ADMM_VOLUME_CONSTRAINTS
#undef X

        // Global solve
        VectorXr b = body.M * v_tilde;

#define X(CTYPE, CFIELD) \
        admm_volume_constraint_update_b(body, constraints.CFIELD.data(), constraints.CFIELD.size(), dt, z.data(), u.data(), \
            (glm::rvec3*) x_orig.data(), OUT b.data());
        ADMM_VOLUME_CONSTRAINTS
#undef X

        v.noalias() = body.A_LDLt.solve(b);
        x.noalias() = x_orig + dt*v;

        real primal_res_sq = 0, dual_res_sq = 0;
#define X(CTYPE, CFIELD) \
        admm_volume_constraint_update_residuals(body, constraints.CFIELD.data(), constraints.CFIELD.size(), \
            z_prev.data(), z.data(), (glm::rvec3*)x.data(), INOUT primal_res_sq, INOUT dual_res_sq);
        ADMM_VOLUME_CONSTRAINTS
#undef X

        std::cout << "primal_res = " << sqrt(primal_res_sq) << ", dual_res= " << sqrt(dual_res_sq) << std::endl;
    }
}
}
