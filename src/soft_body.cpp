//
// Created by lasagnaphil on 21. 2. 8..
//

#include "artsim/soft_body.h"
#include "artsim/math/svd.h"
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
    /*
    while (tri[0] > tri[1] || tri[0] > tri[2]) {
        std::swap(tri[0], tri[1]);
        std::swap(tri[1], tri[2]);
    }
     */
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

void SoftBodyData::load(const OBJFile& obj, const SoftBodyProperties& props) {
    this->props = props;
    vertices = obj.vertices;
    tetrahedrons = obj.tetrahedrons;
    generate_surface_triangles();
}

void SoftBodyData::load(const PyMesh::MshLoader& msh, const SoftBodyProperties& props) {
    auto& nodes = msh.get_nodes();
    auto& elems = msh.get_elements();
    std::cout << "nodes=" << nodes.size() << ", elems=" << elems.size() << std::endl;
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
    generate_surface_triangles();
}

void SoftBodyData::generate_surface_triangles() {
#if 1
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
        std::cout << glm::to_string(tri) << ": " << count << std::endl;
    }
#endif
}

void SoftBodyData::precomputation() {
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

    M = SparseMatrix<real>(3*vertices.size(), 3*vertices.size());
    std::unordered_map<glm::ivec2, real> M_triplets_map;
    for (int i = 0; i < tetrahedrons.size(); i++) {
        real m = props.density * W[i] / 20.0;
        glm::ivec4 tet = tetrahedrons[i];
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
    M_LDLt.analyzePattern(M);
    M_LDLt.factorize(M);

    SparseMatrix<real> A = M;
    for (const auto& c : corotational_energy_constraints) {
        glm::ivec4 tet = tetrahedrons[c.tet_id];
        for (int j = 0; j < 4; j++) {
            for (int k = 0; k < 4; k++) {
                real dA = props.dt * props.dt * c.k * glm::dot(D[c.tet_id][j], D[c.tet_id][k]);
                A.coeffRef(3*tet[j]+0, 3*tet[k]+0) += dA;
                A.coeffRef(3*tet[j]+1, 3*tet[k]+1) += dA;
                A.coeffRef(3*tet[j]+2, 3*tet[k]+2) += dA;
            }
        }
    }
    for (const auto& c : volume_preservation_energy_constraints) {
        glm::ivec4 tet = tetrahedrons[c.tet_id];
        for (int j = 0; j < 4; j++) {
            for (int k = 0; k < 4; k++) {
                real dA = props.dt * props.dt * c.k * glm::dot(D[c.tet_id][j], D[c.tet_id][k]);
                A.coeffRef(3*tet[j]+0, 3*tet[k]+0) += dA;
                A.coeffRef(3*tet[j]+1, 3*tet[k]+1) += dA;
                A.coeffRef(3*tet[j]+2, 3*tet[k]+2) += dA;
            }
        }
    }

    SparseMatrix<real> A_prime = A;
    A_LDLt.analyzePattern(A_prime);
    A_LDLt.factorize(A_prime);
    real reg = 1e-6;
    bool success = true;
    while (A_LDLt.info() != Success) {
        reg *= 10;
        for(int i = 0; i < 3*vertices.size(); i++) {
            A_prime.coeffRef(i, i) = A.coeffRef(i, i) + reg;
        }
        A_LDLt.factorize(A_prime);
        success = false;
    }
    if (!success) {
        std::cout << "factorize failure (Damping : " << reg << " )" << std::endl;
    }
}

void SoftBodyData::add_corotational_energy(int tet_id, real k, real mu, real lambda) {
    CorotationalEnergyConstraint constraint;
    constraint.tet_id = tet_id;
    constraint.mu = mu;
    constraint.lambda = lambda;
    constraint.k = k;
    corotational_energy_constraints.push_back(constraint);
}

void SoftBodyData::add_corotational_energy_full_body(real k, real mu, real lambda) {
    for (int i = 0; i < tetrahedrons.size(); i++) {
        add_corotational_energy(i, k, mu, lambda);
    }
}

void SoftBodyData::add_neohookean_energy(int tet_id, real k, real mu, real lambda) {
    NeoHookeanEnergyConstraint constraint;
    constraint.tet_id = tet_id;
    constraint.mu = mu;
    constraint.lambda = lambda;
    constraint.k = k;
    neohookean_energy_constraints.push_back(constraint);
}

void SoftBodyData::add_neohookean_energy_full_body(real k, real mu, real lambda) {
    for (int i = 0; i < tetrahedrons.size(); i++) {
        add_neohookean_energy(i, k, mu, lambda);
    }
}

void SoftBodyData::add_volume_preservation_energy(int tet_id, real k, real sigma_min, real sigma_max) {
    VolumePreservationEnergyConstraint constraint;
    constraint.tet_id = tet_id;
    constraint.k = k;
    constraint.sigma_min = sigma_min;
    constraint.sigma_max = sigma_max;
    volume_preservation_energy_constraints.push_back(constraint);
}

void SoftBodyData::add_volume_preservation_energy_full_body(real k, real sigma_min, real sigma_max) {
    for (int i = 0; i < tetrahedrons.size(); i++) {
        add_volume_preservation_energy(i, k, sigma_min, sigma_max);
    }
}

glm::tmat3x3<real> projection(const glm::tmat3x3<real>& F, const CorotationalEnergyConstraint& c) {
    auto F_svd = glmx::svd(F);
    return F_svd.U * glm::transpose(F_svd.V);
}

glm::tmat3x3<real> proximal(const glm::tmat3x3<real>& F, const CorotationalEnergyConstraint& c, real tau) {
    auto F_svd = glmx::svd(F);
    real A_diag = 2*c.mu + c.lambda + tau;
    glm::tvec3<real> sigma = glm::tvec3<real>(F_svd.Sigma[0][0], F_svd.Sigma[1][1], F_svd.Sigma[2][2]);
    glm::tmat3x3<real> A(A_diag, c.lambda, c.lambda, c.lambda, A_diag, c.lambda, c.lambda, c.lambda, A_diag);
    glm::tvec3<real> b(2*c.mu + c.lambda + tau*sigma.x,
                 2*c.mu + c.lambda + tau*sigma.y,
                 2*c.mu + c.lambda + tau*sigma.z);
    sigma = inverse(A) * b;
    glm::tmat3x3<real> Sigma(sigma.x, 0, 0, 0, sigma.y, 0, 0, 0, sigma.z);
    return F_svd.U * Sigma * glm::transpose(F_svd.V);
}

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

glm::tmat3x3<real> projection(const glm::tmat3x3<real>& F, const NeoHookeanEnergyConstraint& c) {
    // TODO
    fprintf(stderr, "Unimplemented!\n");
    exit(EXIT_FAILURE);
}

glm::tmat3x3<real> proximal(const glm::tmat3x3<real>& F, const NeoHookeanEnergyConstraint& c, real tau) {
    auto F_svd = glmx::svd(F);
    auto S_star = glm::tvec3<real>(F_svd.Sigma[0][0], F_svd.Sigma[1][1], F_svd.Sigma[2][2]);
    auto S = S_star;
    for (int i = 0; i < 5; i++) {
        real J = glm::log(S[0]*S[1]*S[2]);
        glm::tvec3<real> grad;
        grad[0] = c.mu*(S[0] - real(1)/S[0]) + c.lambda/S[0] * J + tau*(S[0] - S_star[0]);
        grad[1] = c.mu*(S[1] - real(1)/S[1]) + c.lambda/S[1] * J + tau*(S[1] - S_star[1]);
        grad[2] = c.mu*(S[2] - real(1)/S[2]) + c.lambda/S[2] * J + tau*(S[2] - S_star[2]);
        glmx::tsmat3x3<real> H;
        H.xx = c.mu + (c.mu + c.lambda)/(S[0]*S[0]) - c.lambda/(S[0]*S[0]) * J + tau;
        H.yy = c.mu + (c.mu + c.lambda)/(S[1]*S[1]) - c.lambda/(S[1]*S[1]) * J + tau;
        H.zz = c.mu + (c.mu + c.lambda)/(S[2]*S[2]) - c.lambda/(S[2]*S[2]) * J + tau;
        H.yz = c.lambda / (S[1]*S[2]);
        H.zx = c.lambda / (S[2]*S[0]);
        H.xy = c.lambda / (S[0]*S[1]);
        S -= glmx::inverse(H) * grad;
    }
    glm::tmat3x3<real> Sigma(S.x, 0, 0, 0, S.y, 0, 0, 0, S.z);
    return F_svd.U * Sigma * glm::transpose(F_svd.V);
}

glm::tmat3x3<real> projection(const glm::tmat3x3<real>& F, const VolumePreservationEnergyConstraint& c) {
    auto F_svd = glmx::svd(F);
    auto S = glm::tvec3<real>(F_svd.Sigma[0][0], F_svd.Sigma[1][1], F_svd.Sigma[2][2]);
    auto S_star = calc_S_star(S, c.sigma_min, c.sigma_max);
    auto Sigma = glm::tmat3x3<real>(S_star[0], 0, 0, 0, S_star[1], 0, 0, 0, S_star[2]);
    return F_svd.U * Sigma * glm::transpose(F_svd.V);
}

glm::tmat3x3<real> proximal(const glm::tmat3x3<real>& F, const VolumePreservationEnergyConstraint& c, real tau) {
    return projection(F, c);
}

template <class Constraint>
void projective_dynamics_volume_constraint_local_solve(
        const SoftBodyData& body, const Constraint* constraints, uint32_t num_constraints,
        const glm::tvec3<real>* V,
        OUT glm::tmat3x3<real>* p) {

#pragma omp parallel for schedule(static)
    for (int cidx = 0; cidx < num_constraints; cidx++) {
        auto& c = constraints[cidx];
        glm::ivec4 tet = body.tetrahedrons[c.tet_id];
        glm::tmat3x3<real> D_s(V[tet[0]] - V[tet[3]], V[tet[1]] - V[tet[3]], V[tet[2]] - V[tet[3]]);
        glm::tmat3x3<real> F = D_s * body.B_m[c.tet_id];
        p[c.tet_id] = projection(F, c);
    }
}

template <class Constraint>
void admm_volume_constraint_local_solve(
        const SoftBodyData& body, const Constraint* constraints, uint32_t num_constraints,
        const glm::tvec3<real>* V,
        OUT glm::tmat3x3<real>* z, OUT glm::tmat3x3<real>* u, OUT glm::tmat3x3<real>* p) {

#pragma omp parallel for schedule(static)
    for (int cidx = 0; cidx < num_constraints; cidx++) {
        auto& c = constraints[cidx];
        glm::ivec4 tet = body.tetrahedrons[c.tet_id];
        glm::tmat3x3<real> D_s(V[tet[0]] - V[tet[3]], V[tet[1]] - V[tet[3]], V[tet[2]] - V[tet[3]]);
        glm::tmat3x3<real> F = D_s * body.B_m[c.tet_id] + u[c.tet_id];
        z[c.tet_id] = proximal(F, c, c.k / body.W[c.tet_id]);
        u[c.tet_id] = F - z[c.tet_id];
        p[c.tet_id] = z[c.tet_id] - u[c.tet_id];
    }
}

template <class Constraint>
void global_solve_modify_b(const SoftBodyData& body, const Constraint* constraints, uint32_t num_constraints,
                         const glm::tmat3x3<real>* p, INOUT real* b) {
    for (int cidx = 0; cidx < num_constraints; cidx++) {
        auto& c = constraints[cidx];
        glm::ivec4 tet = body.tetrahedrons[c.tet_id];
        auto& p_mat = p[c.tet_id];
        auto& D_i = body.D[c.tet_id];
        real k_s = body.props.dt * body.props.dt * c.k;
        for (int j = 0; j < 4; j++) {
            glm::tvec3<real> db = k_s * p_mat * D_i[j];
            b[3*tet[j]+0] += db[0];
            b[3*tet[j]+1] += db[1];
            b[3*tet[j]+2] += db[2];
        }
    }
}

void soft_body_dynamics(const SoftBodyData& body, FEMAlgorithmType alg_type, real dt, const real* f,
                        INOUT real* pos, INOUT real* vel) {
    Map<VectorXr> x(pos, 3*body.vertices.size());
    Map<VectorXr> v(vel, 3*body.vertices.size());
    Map<const VectorXr> f_ext(f, 3*body.vertices.size());
    VectorXr x_orig = x;
    x += dt * v + dt*dt * body.M_LDLt.solve(f_ext);

    std::vector<glm::tmat3x3<real>> u(body.tetrahedrons.size(), glm::tmat3x3<real>(0.0));
    std::vector<glm::tmat3x3<real>> z(body.tetrahedrons.size());
    std::vector<glm::tmat3x3<real>> p(body.tetrahedrons.size());

    glm::tvec3<real>* V = (glm::tvec3<real>*) pos;

    for (int iter = 0; iter < 5; iter++) {
        // Local solve
        if (alg_type == FEMAlgorithmType::ProjectiveDynamics) {
            projective_dynamics_volume_constraint_local_solve(
                    body, body.corotational_energy_constraints.data(), body.corotational_energy_constraints.size(), V,
                    OUT p.data());
            projective_dynamics_volume_constraint_local_solve(
                    body, body.neohookean_energy_constraints.data(), body.neohookean_energy_constraints.size(), V,
                    OUT p.data());
            projective_dynamics_volume_constraint_local_solve(
                    body, body.volume_preservation_energy_constraints.data(), body.volume_preservation_energy_constraints.size(), V,
                    OUT p.data());
        }
        else if (alg_type == FEMAlgorithmType::ADMM) {
            admm_volume_constraint_local_solve(
                    body, body.corotational_energy_constraints.data(), body.corotational_energy_constraints.size(), V,
                    OUT z.data(), OUT u.data(), OUT p.data());
            admm_volume_constraint_local_solve(
                    body, body.neohookean_energy_constraints.data(), body.neohookean_energy_constraints.size(), V,
                    OUT z.data(), OUT u.data(), OUT p.data());
            admm_volume_constraint_local_solve(
                    body, body.volume_preservation_energy_constraints.data(), body.volume_preservation_energy_constraints.size(), V,
                    OUT z.data(), OUT u.data(), OUT p.data());

        }

        // Global solve
        VectorXr b = body.M * x;
        global_solve_modify_b(
                body, body.corotational_energy_constraints.data(), body.corotational_energy_constraints.size(), p.data(),
                OUT b.data());
        global_solve_modify_b(
                body, body.neohookean_energy_constraints.data(), body.neohookean_energy_constraints.size(), p.data(),
                OUT b.data());
        global_solve_modify_b(
                body, body.volume_preservation_energy_constraints.data(), body.volume_preservation_energy_constraints.size(), p.data(),
                OUT b.data());

        x = body.A_LDLt.solve(b);
    }
    v = (x - x_orig) / dt;
}

}
