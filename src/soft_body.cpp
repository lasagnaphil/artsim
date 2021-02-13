//
// Created by lasagnaphil on 21. 2. 8..
//

#include "artsim/soft_body.h"
#include "artsim/math/svd.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/IterativeLinearSolvers>
#include <unordered_map>
#include <glm/gtx/hash.hpp>

using namespace Eigen;
using Matrix3dr = Matrix<double, 3, 3, RowMajor>;

namespace artsim {

void OBJFile::load(const char* filename) {
    std::ifstream ifs(filename);
    if (!(ifs.is_open())) {
        std::cout << "Can't read file " << filename << std::endl;
        return;
    }
    std::string str;
    std::string index;
    std::stringstream ss;

    while (!ifs.eof()) {
        str.clear();
        index.clear();
        ss.clear();

        std::getline(ifs, str);
        ss.str(str);
        ss >> index;

        if (index == "v") {
            glm::dvec3 v;
            ss >> v[0] >> v[1] >> v[2];
            vertices.push_back(v);
        }
        else if (index == "vn") {
            glm::dvec3 vn;
            ss >> vn[0] >> vn[1] >> vn[2];
            normals.push_back(vn);
        }
        else if (index == "vt") {
            glm::dvec2 vt;
            ss >> vt[0] >> vt[1];
            uvs.push_back(vt);
        }
        else if (index == "f") {
            glm::ivec3 fi, ft, fn;
            int res;
            res = sscanf(str.c_str(), "f %d %d %d", &fi.x, &fi.y, &fi.z);
            if (res == 3) {
                triangle_vertices.push_back(fi);
                continue;
            }
            res = sscanf(str.c_str(), "f %d/%d/%d %d/%d/%d %d/%d/%d",
                   &fi.x,&ft.x,&fn.x,
                   &fi.y,&ft.y,&fn.y,
                   &fi.z,&ft.z,&fn.z);
            if (res == 9) {
                triangle_vertices.push_back(fi);
                triangle_uvs.push_back(ft);
                triangle_normals.push_back(fn);
                continue;
            }
            fprintf(stderr, "Error while parsing OBJ file!\n");
        } else if (index == "t") {
            glm::ivec4 t;
            ss >> t[0] >> t[1] >> t[2] >> t[3];
            tetrahedrons.push_back(t);
        }
    }
    ifs.close();
}

glm::ivec3 reorder_tri_indices(glm::ivec3 tri) {
    if (tri[1] > tri[2]) std::swap(tri[1], tri[2]);
    if (tri[0] > tri[1]) std::swap(tri[0], tri[1]);
    if (tri[1] > tri[2]) std::swap(tri[1], tri[2]);
    return tri;
}

void SoftBodyData::load(const OBJFile& obj, const SoftBodyProperties& props) {
    this->props = props;
    vertices = obj.vertices;
    if (obj.triangle_vertices.empty()) {
        std::unordered_map<glm::ivec3, int> tri_overlaps;
        auto insert_triangle = [&](glm::ivec3 tri) {
            auto it = tri_overlaps.find(tri);
            if (it == tri_overlaps.end()) {
                tri_overlaps.insert({tri, 1});
            }
            else {
                it->second++;
            }
        };
        for (int i = 0; i < obj.tetrahedrons.size(); i++) {
            auto i0 = obj.tetrahedrons[i].x;
            auto i1 = obj.tetrahedrons[i].y;
            auto i2 = obj.tetrahedrons[i].z;
            auto i3 = obj.tetrahedrons[i].w;
            insert_triangle(reorder_tri_indices({i0, i1, i2}));
            insert_triangle(reorder_tri_indices({i0, i1, i3}));
            insert_triangle(reorder_tri_indices({i0, i2, i3}));
            insert_triangle(reorder_tri_indices({i1, i2, i3}));
        }
        for (auto& [tri, count] : tri_overlaps) {
            if (count == 1) {
                triangles.push_back(tri);
            }
        }
    }
    else {
        triangles = obj.triangle_vertices;
    }

    tetrahedrons = obj.tetrahedrons;
}

void SoftBodyData::precomputation() {
    B_m.resize(tetrahedrons.size());
    W.resize(tetrahedrons.size());
    D.resize(tetrahedrons.size());
    for (int i = 0; i < tetrahedrons.size(); i++) {
        auto& V = vertices;
        glm::ivec4 tet = tetrahedrons[i];
        glm::dmat3x3 D_m(V[tet[0]] - V[tet[3]], V[tet[1]] - V[tet[3]], V[tet[2]] - V[tet[3]]);
        B_m[i] = glm::inverse(D_m);
        W[i] = glm::determinant(D_m) / 6.0;
        D[i][0] = D_m[0];
        D[i][1] = D_m[1];
        D[i][2] = D_m[2];
        D[i][3] = -D_m[0] - D_m[1] - D_m[2];
    }

    M = SparseMatrix<double>(3*vertices.size(), 3*vertices.size());
    std::unordered_map<glm::ivec2, double> M_triplets_map;
    for (int i = 0; i < tetrahedrons.size(); i++) {
        double m = props.density * W[i] / 20.0;
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
    std::vector<Triplet<double>> M_triplets;
    M_triplets.reserve(M_triplets_map.size());
    for (auto& [k, v] : M_triplets_map) {
        M_triplets.emplace_back(k[0], k[1], v);
    }
    M.setFromTriplets(M_triplets.begin(), M_triplets.end());

    SparseMatrix<double> A = M;
    double k_s = props.dt * props.dt * props.stiffness;
    for (const auto& c : corotational_energy_constraints) {
        glm::ivec4 tet = tetrahedrons[c.tet_id];
        for (int j = 0; j < 4; j++) {
            for (int k = 0; k < 4; k++) {
                double dA = c.k * glm::dot(D[c.tet_id][j], D[c.tet_id][k]);
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
                double dA = c.k * glm::dot(D[c.tet_id][j], D[c.tet_id][k]);
                A.coeffRef(3*tet[j]+0, 3*tet[k]+0) += dA;
                A.coeffRef(3*tet[j]+1, 3*tet[k]+1) += dA;
                A.coeffRef(3*tet[j]+2, 3*tet[k]+2) += dA;
            }
        }
    }

    SparseMatrix<double> A_prime = A;
    A_LDLt.analyzePattern(A_prime);
    A_LDLt.factorize(A_prime);
    double reg = 1e-6;
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

void SoftBodyData::add_corotational_energy(int tet_id, double mu, double lambda, double k) {
    CorotationalEnergyConstraint constraint;
    constraint.tet_id = tet_id;
    constraint.mu = mu;
    constraint.lambda = lambda;
    constraint.k = k;
    corotational_energy_constraints.push_back(constraint);
}

void SoftBodyData::add_corotational_energy_full_body(double mu, double lambda, double k) {
    for (int i = 0; i < tetrahedrons.size(); i++) {
        add_corotational_energy(i, mu, lambda, k);
    }
}

void SoftBodyData::add_volume_preservation_energy(int tet_id, double k) {
    VolumePreservationEnergyConstraint constraint;
    constraint.tet_id = tet_id;
    constraint.k = k;
    volume_preservation_energy_constraints.push_back(constraint);
}

void SoftBodyData::add_volume_preservation_energy_full_body(double k) {
    for (int i = 0; i < tetrahedrons.size(); i++) {
        add_volume_preservation_energy(i, k);
    }
}

glm::dmat3 projection(const glm::dmat3& F, const CorotationalEnergyConstraint& c) {
    auto F_svd = glmx::svd(F);
    return F_svd.U * glm::transpose(F_svd.V);
}

glm::dmat3 proximal(const glm::dmat3& F, const CorotationalEnergyConstraint& c, double tau) {
    auto F_svd = glmx::svd(F);
    double A_diag = 2*c.mu + c.lambda + tau;
    glm::dvec3 sigma = glm::dvec3(F_svd.Sigma[0][0], F_svd.Sigma[1][1], F_svd.Sigma[2][2]);
    glm::dmat3 A(A_diag, c.lambda, c.lambda, c.lambda, A_diag, c.lambda, c.lambda, c.lambda, A_diag);
    glm::dvec3 b(2*c.mu + c.lambda + tau*sigma.x,
                 2*c.mu + c.lambda + tau*sigma.y,
                 2*c.mu + c.lambda + tau*sigma.z);
    sigma = inverse(A) * b;
    glm::dmat3 Sigma(sigma.x, 0, 0, 0, sigma.y, 0, 0, 0, sigma.z);
    return F_svd.U * Sigma * glm::transpose(F_svd.V);
}

glm::dvec3 calc_sigma_star(const glm::dvec3 sigma) {
    // TODO
}

template <class Constraint>
void projective_dynamics_volume_constraint_local_solve(
        const SoftBodyData& body, const Constraint* constraints, uint32_t num_constraints,
        const glm::dvec3* V,
        OUT glm::dmat3x3* p) {

#pragma omp parallel for
    for (int cidx = 0; cidx < num_constraints; cidx++) {
        auto& c = constraints[cidx];
        glm::ivec4 tet = body.tetrahedrons[c.tet_id];
        glm::dmat3x3 D_s(V[tet[0]] - V[tet[3]], V[tet[1]] - V[tet[3]], V[tet[2]] - V[tet[3]]);
        glm::dmat3x3 F = D_s * body.B_m[c.tet_id];
        p[c.tet_id] = projection(F, c);
    }
}

template <class Constraint>
void admm_volume_constraint_local_solve(
        const SoftBodyData& body, const Constraint* constraints, uint32_t num_constraints,
        const glm::dvec3* V,
        OUT glm::dmat3x3* z, OUT glm::dmat3x3* u, OUT glm::dmat3x3* p) {

#pragma omp parallel for
    for (int cidx = 0; cidx < num_constraints; cidx++) {
        auto& c = constraints[cidx];
        glm::ivec4 tet = body.tetrahedrons[c.tet_id];
        glm::dmat3x3 D_s(V[tet[0]] - V[tet[3]], V[tet[1]] - V[tet[3]], V[tet[2]] - V[tet[3]]);
        glm::dmat3x3 F = D_s * body.B_m[c.tet_id] + u[c.tet_id];
        z[c.tet_id] = proximal(F, c, c.k / body.W[c.tet_id]);
        u[c.tet_id] = F - z[c.tet_id];
        p[c.tet_id] = z[c.tet_id] - u[c.tet_id];
    }
}

template <class Constraint>
void global_solve_modify_b(const SoftBodyData& body, const Constraint* constraints, uint32_t num_constraints,
                         const glm::dmat3* p, INOUT double* b) {
#pragma omp parallel for
    for (int cidx = 0; cidx < body.corotational_energy_constraints.size(); cidx++) {
        auto& c = body.corotational_energy_constraints[cidx];
        glm::ivec4 tet = body.tetrahedrons[c.tet_id];
        auto& p_mat = p[c.tet_id];
        double k_s = body.props.dt * body.props.dt * body.props.stiffness;
        for (int j = 0; j < 4; j++) {
            b[3*tet[j]+0] += k_s * glm::dot(body.D[c.tet_id][j], glm::dvec3(p_mat[0][0], p_mat[1][0], p_mat[2][0]));
            b[3*tet[j]+1] += k_s * glm::dot(body.D[c.tet_id][j], glm::dvec3(p_mat[0][1], p_mat[1][1], p_mat[2][1]));
            b[3*tet[j]+2] += k_s * glm::dot(body.D[c.tet_id][j], glm::dvec3(p_mat[0][2], p_mat[1][2], p_mat[2][2]));
        }
    }
}

void soft_body_dynamics(const SoftBodyData& body, FEMAlgorithmType alg_type, double dt,
                        INOUT double* pos, INOUT double* vel) {
    Map<VectorXd> x(pos, 3*body.vertices.size());
    Map<VectorXd> v(vel, 3*body.vertices.size());
    VectorXd x_orig = x;
    x.noalias() += v * dt;

    std::vector<glm::dmat3x3> u(body.tetrahedrons.size(), glm::dmat3x3(0.0));
    std::vector<glm::dmat3x3> z(body.tetrahedrons.size());
    std::vector<glm::dmat3x3> p(body.tetrahedrons.size());

    glm::dvec3* V = (glm::dvec3*) pos;

    for (int iter = 0; iter < 10; iter++) {
        // Local solve
        projective_dynamics_volume_constraint_local_solve(
                body, body.corotational_energy_constraints.data(), body.corotational_energy_constraints.size(), V,
                OUT p.data());

        // Global solve
        VectorXd b = body.M * x;
        global_solve_modify_b(
                body, body.corotational_energy_constraints.data(), body.corotational_energy_constraints.size(), p.data(),
                OUT b.data());

        x.noalias() = body.A_LDLt.solve(b);
    }
    v.noalias() = (x - x_orig) / dt;
}

}
