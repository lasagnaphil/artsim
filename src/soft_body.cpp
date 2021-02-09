//
// Created by lasagnaphil on 21. 2. 8..
//

#include "artsim/soft_body.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/IterativeLinearSolvers>

using namespace Eigen;
using Matrix3dr = Matrix<double, 3, 3, RowMajor>;

namespace artsim {

void SoftBody::load_from_mesh(const char* filename) {
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
            Vector3d v;
            ss >> v(0) >> v(1) >> v(2);
            vertices.push_back(v);
        } else if (index == "f") {
            Vector3i f;
            ss >> f(0) >> f(1) >> f(2);
            triangles.push_back(f);
        } else if (index == "t") {
            Vector4i t;
            ss >> t(0) >> t(1) >> t(2) >> t(3);
            tetrahedrons.push_back(t);
        }

    }
    ifs.close();
}

void SoftBody::setup() {
    B_m.resize(tetrahedrons.size());
    for (int i = 0; i < tetrahedrons.size(); i++) {
        auto& V = vertices;
        Vector4i tet = tetrahedrons[i];
        Matrix3dr D_m;
        D_m << V[tet(0)](0) - V[tet(3)](0), V[tet(1)](0) - V[tet(3)](0), V[tet(2)](0) - V[tet(3)](0),
               V[tet(0)](1) - V[tet(3)](1), V[tet(1)](1) - V[tet(3)](1), V[tet(2)](1) - V[tet(3)](1),
               V[tet(0)](2) - V[tet(3)](2), V[tet(1)](2) - V[tet(3)](2), V[tet(2)](2) - V[tet(3)](2);
        B_m[i] = D_m.inverse();
        W[i] = D_m.determinant() / 6.0;
    }
    M.resize(vertices.size(), vertices.size());
    M.setZero();
    for (int i = 0; i < tetrahedrons.size(); i++) {
        double m = density * W[i] / 20.0;
        Vector4i tet = tetrahedrons[i];
        for (int j = 0; j < 4; j++) {
            for (int k = 0; k < 4; k++) {
                if (j == k) {
                    M(3*tet(j), 3*tet(k)) = M(3*tet(j)+1, 3*tet(k)+1) = M(3*tet(j)+2, 3*tet(k)+2) = 2.0*m;
                }
                else {
                    M(3*tet(j), 3*tet(k)) = M(3*tet(j)+1, 3*tet(k)+1) = M(3*tet(j)+2, 3*tet(k)+2) = m;
                }
            }
        }
    }
}

void soft_body_dynamics(const SoftBody& body, FEMAlgorithmType alg_type, double dt, OUT double* vertices) {
    Map<VectorXd> x(vertices, 3*body.vertices.size());
    std::vector<Matrix3dr> u(body.tetrahedrons.size(), Matrix3d::Identity());
    std::vector<Matrix3dr> z(body.tetrahedrons.size());

    // TODO: populate weight matrix with something sensible
    //       for now, just use stiffness value as in Projective Dynamics
    // std::vector<Matrix3dr> W(body.tetrahedrons.size());

    Map<Matrix<double, Dynamic, 3, RowMajor>> V(vertices, body.vertices.size(), 3);

// #pragma omp parallel for
    for (int i = 0; i < body.tetrahedrons.size(); i++) {
        Vector4i tet = body.tetrahedrons[i];
        Matrix3dr D_s;
        D_s << V(tet(0), 0) - V(tet(3), 0), V(tet(1), 0) - V(tet(3), 0), V(tet(2), 0) - V(tet(3), 0),
               V(tet(0), 1) - V(tet(3), 1), V(tet(1), 1) - V(tet(3), 1), V(tet(2), 1) - V(tet(3), 1),
               V(tet(0), 2) - V(tet(3), 2), V(tet(1), 2) - V(tet(3), 2), V(tet(2), 2) - V(tet(3), 2);

        Matrix3dr F = D_s * body.B_m[i] + u[i];

        if (alg_type == FEMAlgorithmType::ProjectiveDynamics) {
            // TODO: If we have no constraints, the equation simply reduces to z = F, u = 0
            // Matrix3dr p = proj(F);
            // z[i].noalias() = 0.5 * (p + F);
            z[i].noalias() = F;
            u[i].noalias() = Matrix3dr::Zero();
        }
        else if (alg_type == FEMAlgorithmType::ADMM) {
            // TODO: Apply proximal operator to F, given material
            // z[i].noalias() = proximal(F);
            u[i].noalias() = F - z[i];
        }
    }

    MatrixXd A = body.M;
    MatrixXd b = body.M * x;

    // TODO: We can do precomputation on matrix A beforehand!
    for (int i = 0; i < body.tetrahedrons.size(); i++) {
        Vector4i tet = body.tetrahedrons[i];
        Matrix<double, 9, 12> D_i;
        // TODO: Calculate D_i based on body.B_m[i]
        // D_i = ...;

        Matrix<double, 12, 12> dM = dt * dt * body.stiffness * D_i.transpose() * D_i;
        for (int j = 0; j < 4; j++) {
            for (int k = 0; k < 4; k++) {
                A.block(3*tet(j), 3*tet(k), 3, 3) += dM.block(3*j, 3*k, 3, 3);
            }
        }

        Matrix3dr p = z[i] - u[i];
        Map<Matrix<double, 9, 1>> p_vec(p.data());
        Matrix<double, 12, 1> dMx = dt * dt * body.stiffness * D_i.transpose() * p_vec;
        for (int j = 0; j < 4; j++) {
            b.middleRows<3>(3*tet(j)) += dMx.middleRows<3>(3*j);
        }
    }

    // TODO: We can also do precomutation on the factorization of matrix A beforehand!
    ConjugateGradient<MatrixXd, Lower|Upper> cg;
    cg.compute(A);
    x = cg.solve(b);
}

}
