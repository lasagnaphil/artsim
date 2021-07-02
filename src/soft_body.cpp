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
#include <unordered_set>
#include <deque>
#include <glm/gtx/hash.hpp>
#include <glm/gtx/string_cast.hpp>
#include <fmt/core.h>
#include <BulletCollision/CollisionShapes/btTriangleMesh.h>
#include <BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h>

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
}

void gen_edges_from_surface_tri_mesh(const std::vector<glm::ivec3>& triangles,
                                     OUT std::vector<glm::ivec2>& edges) {
    std::unordered_set<glm::ivec2> edge_set;
    for (auto& tri : triangles) {
        edge_set.insert({tri[0], tri[1]});
        edge_set.insert({tri[1], tri[2]});
        edge_set.insert({tri[2], tri[0]});
    }
    edges.insert(edges.end(), edge_set.begin(), edge_set.end());
}

void gen_edges_from_tet_mesh(const std::vector<glm::ivec4>& tets,
                             OUT std::vector<glm::ivec2>& edges) {
    std::unordered_set<glm::ivec2> edge_set;
    for (auto& tet : tets) {
        edge_set.insert({tet[0], tet[1]});
        edge_set.insert({tet[1], tet[2]});
        edge_set.insert({tet[2], tet[3]});
        edge_set.insert({tet[3], tet[0]});
    }
    edges.insert(edges.end(), edge_set.begin(), edge_set.end());
}

void SoftBodyData::load(const TetMesh& mesh, const SoftBodyProperties& props) {
    this->props = props;
    vertices = mesh.vertices;
    tetrahedrons = mesh.tetrahedrons;
    gen_surface_triangles_from_tet_mesh(tetrahedrons, OUT surface_triangles);
    gen_edges_from_surface_tri_mesh(surface_triangles, OUT surface_edges);

    B_m.resize(tetrahedrons.size());
    W.resize(tetrahedrons.size());
    D.resize(tetrahedrons.size());
    for (int i = 0; i < tetrahedrons.size(); i++) {
        auto& V = vertices;
        glm::ivec4& tet = tetrahedrons[i];
        glm::tmat3x3<real> D_m(V[tet[0]] - V[tet[3]], V[tet[1]] - V[tet[3]], V[tet[2]] - V[tet[3]]);
        W[i] = glm::determinant(D_m) / 6.0;
        if (W[i] < 0) {
            W[i] = -W[i];
            std::swap(tet[2], tet[3]);
            D_m = glm::tmat3x3<real>(V[tet[0]] - V[tet[3]], V[tet[1]] - V[tet[3]], V[tet[2]] - V[tet[3]]);
        }
        B_m[i] = glm::inverse(D_m);
        glm::tmat3x3<real> D_i = glm::transpose(B_m[i]);
        D[i][0] = D_i[0];
        D[i][1] = D_i[1];
        D[i][2] = D_i[2];
        D[i][3] = -D_i[0] - D_i[1] - D_i[2];
    }

    tetrahedral_mesh_mass_matrix(vertices.size(), props.density,
                                 tetrahedrons.data(), tetrahedrons.size(), W.data(),
                                 OUT M);
    M_LDLt.analyzePattern(M);
    M_LDLt.factorize(M);
}

void SoftBodyData::load(const PyMesh::MshLoader& msh, const SoftBodyProperties& props) {
    this->props = props;
    auto& nodes = msh.get_nodes();
    auto& elems = msh.get_elements();
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
    gen_surface_triangles_from_tet_mesh(tetrahedrons, OUT surface_triangles);
}

btTriangleIndexVertexArray SoftBodyData::create_bullet_surface_trimesh() {
    btIndexedMesh imesh;
    imesh.m_numTriangles = surface_triangles.size();
    imesh.m_triangleIndexBase = reinterpret_cast<const unsigned char*>(surface_triangles.data());
    imesh.m_triangleIndexStride = 3 * sizeof(int);
    imesh.m_numVertices = vertices.size();
    imesh.m_vertexBase = reinterpret_cast<const unsigned char*>(vertices.data());
    imesh.m_vertexStride = 3 * sizeof(real);
    btTriangleIndexVertexArray trimesh;
    trimesh.addIndexedMesh(imesh);
    return trimesh;
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

template <>
void update_system_matrix(SoftBodyData& body, const PDConstraints& constraints, real dt, OUT SparseMatrix<real>& L, OUT SparseMatrix<real>& A) {
    L = body.M;
    for (int k = 0; k < L.outerSize(); ++k) {
        for (SparseMatrix<real>::InnerIterator it(L, k); it; ++it) {
            it.valueRef() = 0;
        }
    }
    A = body.M;
#define X(CTYPE, CFIELD) \
    for (const auto& c : constraints.CFIELD) { \
        glm::ivec4 tet = body.tetrahedrons[c.tet_id]; \
        for (int j = 0; j < 4; j++) { \
            for (int k = 0; k < 4; k++) { \
                real dL = c.k * body.W[c.tet_id] * glm::dot(body.D[c.tet_id][j], body.D[c.tet_id][k]); \
                L.coeffRef(3*tet[j]+0, 3*tet[k]+0) += dL; \
                L.coeffRef(3*tet[j]+1, 3*tet[k]+1) += dL; \
                L.coeffRef(3*tet[j]+2, 3*tet[k]+2) += dL; \
                real dA = dt * dt * dL; \
                A.coeffRef(3*tet[j]+0, 3*tet[k]+0) += dA; \
                A.coeffRef(3*tet[j]+1, 3*tet[k]+1) += dA; \
                A.coeffRef(3*tet[j]+2, 3*tet[k]+2) += dA; \
            } \
        } \
    }
    PD_VOLUME_CONSTRAINTS
#undef X
    for (const auto& c : constraints.positional) {
        real dL = c.k;
        L.coeffRef(3*c.vert_id+0, 3*c.vert_id+0) += dL;
        L.coeffRef(3*c.vert_id+1, 3*c.vert_id+1) += dL;
        L.coeffRef(3*c.vert_id+2, 3*c.vert_id+2) += dL;
        real dA = dt * dt * dL;
        A.coeffRef(3*c.vert_id+0, 3*c.vert_id+0) += dA;
        A.coeffRef(3*c.vert_id+1, 3*c.vert_id+1) += dA;
        A.coeffRef(3*c.vert_id+2, 3*c.vert_id+2) += dA;
    }
    for (const auto& c : constraints.soft_rigid_collision) {
        real dL = c.k;
        L.coeffRef(3*c.vert_id+0, 3*c.vert_id+0) += dL;
        L.coeffRef(3*c.vert_id+1, 3*c.vert_id+1) += dL;
        L.coeffRef(3*c.vert_id+2, 3*c.vert_id+2) += dL;
        real dA = dt * dt * dL;
        A.coeffRef(3*c.vert_id+0, 3*c.vert_id+0) += dA;
        A.coeffRef(3*c.vert_id+1, 3*c.vert_id+1) += dA;
        A.coeffRef(3*c.vert_id+2, 3*c.vert_id+2) += dA;
    }
}

template <>
void update_system_matrix(SoftBodyData& body, const ADMMConstraints& constraints, real dt, OUT SparseMatrix<real>& L, OUT SparseMatrix<real>& A) {
    L = body.M;
    for (int k = 0; k < L.outerSize(); ++k) {
        for (SparseMatrix<real>::InnerIterator it(L, k); it; ++it) {
            it.valueRef() = 0;
        }
    }
    A = body.M;
#define X(CTYPE, CFIELD) \
    for (const auto& c : constraints.CFIELD) { \
        glm::ivec4 tet = body.tetrahedrons[c.tet_id]; \
        for (int j = 0; j < 4; j++) { \
            for (int k = 0; k < 4; k++) { \
                real dL = c.k * body.W[c.tet_id] * glm::dot(body.D[c.tet_id][j], body.D[c.tet_id][k]); \
                L.coeffRef(3*tet[j]+0, 3*tet[k]+0) += dL; \
                L.coeffRef(3*tet[j]+1, 3*tet[k]+1) += dL; \
                L.coeffRef(3*tet[j]+2, 3*tet[k]+2) += dL; \
                real dA = dt * dt * dL; \
                A.coeffRef(3*tet[j]+0, 3*tet[k]+0) += dA; \
                A.coeffRef(3*tet[j]+1, 3*tet[k]+1) += dA; \
                A.coeffRef(3*tet[j]+2, 3*tet[k]+2) += dA; \
            } \
        } \
    }
    ADMM_VOLUME_CONSTRAINTS
#undef X
    for (const auto& c : constraints.positional) {
        real dL = c.k;
        L.coeffRef(3*c.vert_id+0, 3*c.vert_id+0) += dL;
        L.coeffRef(3*c.vert_id+1, 3*c.vert_id+1) += dL;
        L.coeffRef(3*c.vert_id+2, 3*c.vert_id+2) += dL;
        real dA = dt * dt * dL;
        A.coeffRef(3*c.vert_id+0, 3*c.vert_id+0) += dA;
        A.coeffRef(3*c.vert_id+1, 3*c.vert_id+1) += dA;
        A.coeffRef(3*c.vert_id+2, 3*c.vert_id+2) += dA;
    }
}

template <class Constraints>
void soft_body_precomputation(SoftBodyData& body, const Constraints& constraints, real dt, OUT SoftBodyPrecalcData& precalc) {
    update_system_matrix(body, constraints, dt, OUT precalc.L, OUT precalc.A);

    precalc.A_LDLt.analyzePattern(precalc.A);
    precalc.A_LDLt.factorize(precalc.A);

    precalc.L_LDLt.analyzePattern(precalc.L);
    precalc.L_LDLt.factorize(precalc.L);
}

template void soft_body_precomputation<PDConstraints>(SoftBodyData& body, const PDConstraints& constraints, real dt,
        OUT SoftBodyPrecalcData& precalc);
template void soft_body_precomputation<ADMMConstraints>(SoftBodyData& body, const ADMMConstraints& constraints, real dt,
        OUT SoftBodyPrecalcData& precalc);

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

void soft_body_calc_deformation_field(const SoftBodyData& body, const glm::rvec3* x, OUT glm::rmat3* F) {
    int num_tets = body.tetrahedrons.size();
    for (int tidx = 0; tidx < num_tets; tidx++) {
        auto& tet = body.tetrahedrons[tidx];
        F[tidx] = glm::rmat3(x[tet[0]] - x[tet[3]], x[tet[1]] - x[tet[3]], x[tet[2]] - x[tet[3]]) * body.B_m[tidx];
    }
}

void soft_body_calc_deformation_field(const SoftBodyData& body, const glm::rvec3* x, const glm::rmat3* u, OUT glm::rmat3* F) {
    int num_tets = body.tetrahedrons.size();
    for (int tidx = 0; tidx < num_tets; tidx++) {
        auto& tet = body.tetrahedrons[tidx];
        glm::rmat3 Dx = glm::rmat3(x[tet[0]] - x[tet[3]], x[tet[1]] - x[tet[3]], x[tet[2]] - x[tet[3]]) * body.B_m[tidx];
        F[tidx] = Dx + u[tidx];
    }
}

template <class Constraint>
void projective_dynamics_volume_constraint_local_solve(
        const SoftBodyData& body, const Constraint* constraints, uint32_t num_constraints,
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
        const SoftBodyData&, const CTYPE*, uint32_t, \
        const glmx::SVD_mats<real>*, OUT glm::tmat3x3<real>*);
PD_VOLUME_CONSTRAINTS
#undef X

template <class Constraint>
void projective_dynamics_volume_constraint_update_b(
        const SoftBodyData& body, const Constraint* constraints, uint32_t num_constraints, real dt,
        const glm::tmat3x3<real>* p,
        INOUT real* b) {

    for (int cidx = 0; cidx < num_constraints; cidx++) {
        auto& c = constraints[cidx];
        glm::ivec4 tet = body.tetrahedrons[c.tet_id];
        auto& D_i = body.D[c.tet_id];
        real k_s = c.k * dt * dt * body.W[c.tet_id];
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
        const SoftBodyData& body, const CTYPE* constraints, uint32_t num_constraints, real dt, \
        const glm::tmat3x3<real>* F, \
        INOUT real* b);
PD_VOLUME_CONSTRAINTS
#undef X

void projective_dynamics_positional_constraint_update_b(
        const SoftBodyData& body, const PositionalConstraint* constraints, uint32_t num_constraints, real dt,
        INOUT real* b) {
    for (int cidx = 0; cidx < num_constraints; cidx++) {
        auto& c = constraints[cidx];
        real k_s = c.k * dt * dt;
        b[3*c.vert_id+0] += k_s * c.target_pos.x;
        b[3*c.vert_id+1] += k_s * c.target_pos.y;
        b[3*c.vert_id+2] += k_s * c.target_pos.z;
    }
}

void projective_dynamics_soft_rigid_collision_constraint_update_b(
        const SoftBodyData& body, const SoftRigidCollisionConstraint* constraints, uint32_t num_constraints, real dt,
        const glm::rvec3* x, INOUT real* b) {
    for (int cidx = 0; cidx < num_constraints; cidx++) {
        auto& c = constraints[cidx];
        real k_s = c.k * dt * dt;
        if (glm::dot(c.normal, x[c.vert_id] - c.closest_point) < 0) {
            b[3*c.vert_id+0] += k_s * c.closest_point.x;
            b[3*c.vert_id+1] += k_s * c.closest_point.y;
            b[3*c.vert_id+2] += k_s * c.closest_point.z;
        }
        else {
            b[3*c.vert_id+0] += k_s * x[c.vert_id].x;
            b[3*c.vert_id+1] += k_s * x[c.vert_id].y;
            b[3*c.vert_id+2] += k_s * x[c.vert_id].z;
        }
    }
}

template <class Constraint>
void admm_volume_constraint_local_solve(
        const SoftBodyData& body, const Constraint* constraints, uint32_t num_constraints,
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
        const SoftBodyData& body, const CTYPE* constraints, uint32_t num_constraints, \
        const glm::rmat3* F, const glmx::SVD_mats<real>* F_svd, \
        OUT glm::rmat3* z, OUT glm::rmat3* u);
ADMM_VOLUME_CONSTRAINTS
#undef X

template <class Constraint>
void admm_volume_constraint_update_b(
        const SoftBodyData& body, const Constraint* constraints, uint32_t num_constraints,
        real dt, const glm::rmat3* z, const glm::rmat3* u,
        INOUT real* b) {

    for (int cidx = 0; cidx < num_constraints; cidx++) {
        auto& c = constraints[cidx];
        glm::ivec4 tet = body.tetrahedrons[c.tet_id];
        auto p = z[c.tet_id] - u[c.tet_id];
        auto& D_i = body.D[c.tet_id];
        real k_s = c.k * dt * dt * body.W[c.tet_id];
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
        const SoftBodyData& body, const CTYPE* constraints, uint32_t num_constraints, \
        real dt, const glm::rmat3* z, const glm::rmat3* u, INOUT real* b);
ADMM_VOLUME_CONSTRAINTS
#undef X

void admm_positional_constraint_update_b(
        const SoftBodyData& body, const PositionalConstraint* constraints, uint32_t num_constraints, real dt,
        INOUT real* b) {
    for (int cidx = 0; cidx < num_constraints; cidx++) {
        auto& c = constraints[cidx];
        real k_s = c.k * dt * dt;
        b[3*c.vert_id+0] += k_s * c.target_pos.x;
        b[3*c.vert_id+1] += k_s * c.target_pos.y;
        b[3*c.vert_id+2] += k_s * c.target_pos.z;
    }
}
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
        dual_res_sq += c.k * glmx::length2(z_next[c.tet_id] - z_prev[c.tet_id]);
    }
}

#define X(CTYPE, CFIELD) \
template void admm_volume_constraint_update_residuals( \
        const SoftBodyData& body, const CTYPE* constraints, uint32_t num_constraints, \
        const glm::tmat3x3<real>* z_prev, const glm::tmat3x3<real>* z_next, const glm::tvec3<real>* x, \
        INOUT real& primal_res_sq, INOUT real& dual_res_sq);
ADMM_VOLUME_CONSTRAINTS
#undef X

void projective_dynamics(const SoftBodyData& body, const SoftBodyPrecalcData& precalc,
                         const PDConstraints& constraints, real dt, int num_iters, const real* f,
                         INOUT real* pos, INOUT real* vel) {
    Map<VectorXr> x(pos, 3*body.vertices.size());
    Map<VectorXr> v(vel, 3*body.vertices.size());
    Map<const VectorXr> f_ext(f, 3*body.vertices.size());
    VectorXr x_orig = x;
    VectorXr x_tilde = x + dt*v + (dt*dt)*body.M_LDLt.solve(f_ext);

    std::vector<glm::tmat3x3<real>> F(body.tetrahedrons.size());
    std::vector<glmx::SVD_mats<real>> F_svd(body.tetrahedrons.size());
    std::vector<glm::tmat3x3<real>> p(body.tetrahedrons.size());

    for (int iter = 0; iter < num_iters; iter++) {
        soft_body_calc_deformation_field(body, (glm::rvec3*) x.data(), OUT F.data());
        glmx::fastsvd(F.data(), body.tetrahedrons.size(), OUT F_svd.data());

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

        projective_dynamics_soft_rigid_collision_constraint_update_b(
                body, constraints.soft_rigid_collision.data(), constraints.soft_rigid_collision.size(), dt,
                (glm::rvec3*)x.data(), INOUT b.data());

        x = precalc.A_LDLt.solve(b);
    }
    v = (x - x_orig) / dt;
}

void projective_dynamics_quasistatic(const SoftBodyData& body, const SoftBodyPrecalcData& precalc,
                                     const PDConstraints& constraints, int num_iters, const real* f,
                                     INOUT real* pos) {
    Map<VectorXr> x(pos, 3*body.vertices.size());
    Map<const VectorXr> f_ext(f, 3*body.vertices.size());

    std::vector<glm::tmat3x3<real>> F(body.tetrahedrons.size());
    std::vector<glmx::SVD_mats<real>> F_svd(body.tetrahedrons.size());
    std::vector<glm::tmat3x3<real>> p(body.tetrahedrons.size());

    for (int iter = 0; iter < num_iters; iter++) {
        soft_body_calc_deformation_field(body, (glm::rvec3*) x.data(), OUT F.data());
        glmx::fastsvd(F.data(), body.tetrahedrons.size(), OUT F_svd.data());

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
        projective_dynamics_soft_rigid_collision_constraint_update_b(
                body, constraints.soft_rigid_collision.data(), constraints.soft_rigid_collision.size(), 1.0,
                (glm::rvec3*) x.data(), INOUT b.data());

        x = precalc.L_LDLt.solve(b);
    }
}

void admm_dynamics(const SoftBodyData& body, const SoftBodyPrecalcData& precalc,
                   const ADMMConstraints& constraints, real dt, int num_iters, const real* f,
                   INOUT real* pos, INOUT real* vel) {

    int num_vertices = body.vertices.size();
    int num_tets = body.tetrahedrons.size();

    Map<VectorXr> x(pos, 3*num_vertices);
    Map<VectorXr> v(vel, 3*num_vertices);
    Map<const VectorXr> f_ext(f, 3*num_vertices);
    VectorXr x_orig = x;
    VectorXr x_tilde = x + dt*v + (dt*dt)*body.M_LDLt.solve(f_ext);
    x = x_tilde;

    std::vector<glm::tmat3x3<real>> u(num_tets, glm::tmat3x3<real>(0.0));
    std::vector<glm::tmat3x3<real>> z(num_tets, glm::tmat3x3<real>(0.0));
    std::vector<glm::tmat3x3<real>> z_prev(num_tets);
    std::vector<glm::tmat3x3<real>> F(num_tets);
    std::vector<glmx::SVD_mats<real>> F_svd(num_tets);

    std::cout << std::endl << "Starting ADMM loop" << std::endl;
    for (int iter = 0; iter < num_iters; iter++) {
        z_prev = z;

        for (int tidx = 0; tidx < num_tets; tidx++) {
            auto& tet = body.tetrahedrons[tidx];
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

        x.noalias() = precalc.A_LDLt.solve(b);

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
        const SoftBodyData& body, const SoftBodyPrecalcData& precalc, const ADMMConstraints& constraints, real dt,
        const VectorXr& x, const VectorXr& x_tilde, OUT glm::rmat3* F, OUT glmx::SVD_mats<real>* F_svd) {
    soft_body_calc_deformation_field(body, (glm::rvec3*) x.data(), OUT F);
    glmx::fastsvd(F, body.tetrahedrons.size(), OUT F_svd);
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
        const SoftBodyData& body, const Constraint* constraints, uint32_t num_constraints, real dt,
        const glm::rmat3* F, const glmx::SVD_mats<real>* F_svd, OUT glm::rvec3* E_grad) {

    real dt_sq = dt * dt;
    for (int cidx = 0; cidx < num_constraints; cidx++) {
        auto& c = constraints[cidx];
        glm::ivec4 tet = body.tetrahedrons[c.tet_id];
        auto& D_i = body.D[c.tet_id];
        glm::rmat3 pk1_tensor = pk1(F[c.tet_id], F_svd[c.tet_id], c);
        real k_s = c.k * dt_sq * body.W[c.tet_id];
        for (int j = 0; j < 4; j++) {
            E_grad[tet[j]] += k_s * (pk1_tensor * D_i[j]);
        }
    }
}

void quasinewton_dynamics_positional_constraint_energy_gradient(
        const SoftBodyData& body, const PositionalConstraint* constraints, uint32_t num_constraints, real dt,
        const glm::rvec3* x, OUT glm::rvec3* E_grad) {
    real dt_sq = dt * dt;
    for (int cidx = 0; cidx < num_constraints; cidx++) {
        auto& c = constraints[cidx];
        E_grad[c.vert_id] += c.k * dt_sq * (x[c.vert_id] - c.target_pos);
    }
}

void quasinewton_dynamics_objective_fn_grad(
        const SoftBodyData& body, const SoftBodyPrecalcData& precalc, const ADMMConstraints& constraints, real dt,
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

    void calc_descent_dir(const SoftBodyData& body, const SoftBodyPrecalcData& precalc,
                          const VectorXr& new_x, const VectorXr& new_grad,
                          OUT VectorXr& r) {
        VectorXr q = -new_grad;

        if (x.empty()) {
            r = precalc.A_LDLt.solve(q);
            x.push_front(new_x);
            return;
        }

        VectorXr s0 = new_x - x[0];
        VectorXr t0 = precalc.A * s0;
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

        r = precalc.A_LDLt.solve(q);
        // TODO: cache this summed value!
        for (int i = t.size()-1; i >= 0; i--) {
            real eta = t[i].dot(r) / rho[i];
            r += s[i] * (zeta[i] - eta);
        }
    }
};

void quasinewton_dynamics(const SoftBodyData& body, const SoftBodyPrecalcData& precalc,
                          const ADMMConstraints& constraints, real dt, int num_iters, const real* f,
                          INOUT real* pos, INOUT real* vel) {
    Map<VectorXr> x(pos, 3*body.vertices.size());
    Map<VectorXr> v(vel, 3*body.vertices.size());
    Map<const VectorXr> f_ext(f, 3*body.vertices.size());
    VectorXr x_orig = x;
    VectorXr x_tilde = x + dt*v + dt*dt*body.M_LDLt.solve(f_ext);

    std::vector<glm::tmat3x3<real>> F(body.tetrahedrons.size());
    std::vector<glmx::SVD_mats<real>> F_svd(body.tetrahedrons.size());
    std::vector<glm::tmat3x3<real>> p(body.tetrahedrons.size());

    // Begin Quasi-Newton solver
    SoftBodyQuasiNewtonHistory hist(5);
    const real gamma = 0.3;
    const int max_bt_iters = 10;
    x = x_tilde;
    real g_x0 = quasinewton_dynamics_objective_fn(
            body, precalc, constraints, dt, x, x_tilde, OUT F.data(), OUT F_svd.data());
    printf("g_x0 = %f\n", g_x0);
    VectorXr g_x0_grad(x.size());
    VectorXr d_x0(x.size());
    VectorXr x_cur;
    for (int k = 1; k <= num_iters; k++) {
        quasinewton_dynamics_objective_fn_grad(body, precalc, constraints, dt, F.data(), F_svd.data(),
                                               x, x_tilde, OUT g_x0_grad);
        // hist.calc_descent_dir(body, precalc, x, g_x0_grad, OUT d_x0);
        d_x0 = -precalc.A_LDLt.solve(g_x0_grad);
        std::cout << d_x0.transpose() << std::endl;
        real alpha = real(1.0);
        real g_x;
        x_cur = x;
        for (int iter = 0; iter < max_bt_iters; iter++) {
            x = x_cur + alpha * d_x0;
            g_x = quasinewton_dynamics_objective_fn(
                    body, precalc, constraints, dt, x, x_tilde, OUT F.data(), OUT F_svd.data());
            real g_x_threshold = g_x0 + gamma * alpha * g_x0_grad.dot(d_x0);
            printf("g_x = %f, g_x_threshold = %f\n", g_x, g_x_threshold);
            if (g_x < g_x_threshold) break;
            alpha = alpha / 2;
        }
    }
    v = (x - x_orig) / dt;
}


}
