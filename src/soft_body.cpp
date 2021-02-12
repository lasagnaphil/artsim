//
// Created by lasagnaphil on 21. 2. 8..
//

#include "artsim/soft_body.h"
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

void SoftBodyData::load(const OBJFile& obj) {
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

void SoftBodyData::precomputation(double dt) {
    this->dt = dt;
    B_m.resize(tetrahedrons.size());
    W.resize(tetrahedrons.size());
    for (int i = 0; i < tetrahedrons.size(); i++) {
        auto& V = vertices;
        glm::ivec4 tet = tetrahedrons[i];
        glm::dmat3x3 D_m(
                V[tet.x].x - V[tet.w].x, V[tet.y].x - V[tet.w].x, V[tet.z].x - V[tet.w].x,
                V[tet.x].y - V[tet.w].y, V[tet.y].y - V[tet.w].y, V[tet.z].y - V[tet.w].y,
                V[tet.x].z - V[tet.w].z, V[tet.y].z - V[tet.w].z, V[tet.z].z - V[tet.w].z);
        B_m[i] = glm::inverse(D_m);
        W[i] = glm::determinant(D_m) / 6.0;
    }

    M = SparseMatrix<double>(3*vertices.size(), 3*vertices.size());
    std::unordered_map<glm::ivec2, double> M_triplets_map;
    for (int i = 0; i < tetrahedrons.size(); i++) {
        double m = density * W[i] / 20.0;
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
    for (int i = 0; i < tetrahedrons.size(); i++) {
        glm::ivec4 tet = tetrahedrons[i];
        glm::dvec3 D_i[4];
        D_i[0] = B_m[i][0];
        D_i[1] = B_m[i][1];
        D_i[2] = B_m[i][2];
        D_i[3] = -B_m[i][0] - B_m[i][1] - B_m[i][2];
        double k_s = dt * dt * stiffness;
        for (int j = 0; j < 4; j++) {
            for (int k = 0; k < 4; k++) {
                double dA = k_s * glm::dot(D_i[j], D_i[k]);
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
            A_prime.coeffRef(3*i+0, 3*i+0) = A.coeffRef(3*i+0, 3*i+0) + reg;
        }
        A_LDLt.factorize(A_prime);
        success = false;
    }
    if (!success) {
        std::cout << "factorize failure (Damping : " << reg << " )" << std::endl;
    }
}

glm::dmat3x3 proj(const glm::dmat3x3& F) {
    // TODO
    return F;
}

glm::dmat3x3 proximal(const glm::dmat3x3& F) {
    // TODO
    return F;
}

void soft_body_dynamics(const SoftBodyData& body, FEMAlgorithmType alg_type, double dt, OUT double* pos, OUT double* vel) {
    Map<VectorXd> x(pos, 3*body.vertices.size());
    Map<VectorXd> v(vel, 3*body.vertices.size());
    VectorXd x_orig = x;
    x.noalias() += v * dt;

    std::vector<glm::dmat3x3> u(body.tetrahedrons.size(), glm::dmat3x3(0.0));
    std::vector<glm::dmat3x3> z(body.tetrahedrons.size());
    std::vector<glm::dmat3x3> p(body.tetrahedrons.size());

    // TODO: populate weight matrix with something sensible
    //       for now, just use stiffness value as in Projective Dynamics
    // std::vector<Matrix3dr> W(body.tetrahedrons.size());

    glm::dvec3* V = (glm::dvec3*) pos;

    for (int iter = 0; iter < 10; iter++) {
        // Local solve
        #pragma omp parallel for
        for (int i = 0; i < body.tetrahedrons.size(); i++) {
            glm::ivec4 tet = body.tetrahedrons[i];
            glm::dmat3x3 D_s(
                    V[tet.x].x - V[tet.w].x, V[tet.y].x - V[tet.w].x, V[tet.z].x - V[tet.w].x,
                    V[tet.x].y - V[tet.w].y, V[tet.y].y - V[tet.w].y, V[tet.z].y - V[tet.w].y,
                    V[tet.x].z - V[tet.w].z, V[tet.y].z - V[tet.w].z, V[tet.z].z - V[tet.w].z);

            if (alg_type == FEMAlgorithmType::ProjectiveDynamics) {
                glm::dmat3x3 F = D_s * body.B_m[i];
                p[i] = proj(F);
            }
            else if (alg_type == FEMAlgorithmType::ADMM) {
                // TODO: Apply proximal operator to F, given material
                glm::dmat3x3 F = D_s * body.B_m[i] + u[i];
                z[i] = proximal(F);
                u[i] = F - z[i];
            }
        }

        // Global solve
        MatrixXd b = body.M * x;

        for (int i = 0; i < body.tetrahedrons.size(); i++) {
            glm::ivec4 tet = body.tetrahedrons[i];
            glm::dvec3 D_i[4];
            D_i[0] = body.B_m[i][0];
            D_i[1] = body.B_m[i][1];
            D_i[2] = body.B_m[i][2];
            D_i[3] = -body.B_m[i][0] - body.B_m[i][1] - body.B_m[i][2];
            double k_s = dt * dt * body.stiffness;
            for (int j = 0; j < 4; j++) {
                b(3*tet[j]+0) += k_s * glm::dot(D_i[j], p[i][0]);
                b(3*tet[j]+1) += k_s * glm::dot(D_i[j], p[i][1]);
                b(3*tet[j]+2) += k_s * glm::dot(D_i[j], p[i][2]);
            }
        }

        x.noalias() = body.A_LDLt.solve(b);
    }
    v.noalias() = (x - x_orig) / dt;
}

}
