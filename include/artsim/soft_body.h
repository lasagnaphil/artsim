//
// Created by lasagnaphil on 21. 2. 8..
//

#ifndef ARTSIM_SOFT_BODY_H
#define ARTSIM_SOFT_BODY_H

#include <glm/vec3.hpp>
#include <vector>
#include <artsim/artsim.h>
#include <artsim/tet_mesh.h>
#include <artsim/math/dynmat.h>
#include <artsim/math/svd.h>
#include <artsim/utils/pymesh/MshLoader.h>
#include <Eigen/SparseCholesky>
#include <BulletCollision/CollisionShapes/btTriangleMesh.h>

class btBvhTriangleMeshShape;

namespace artsim {

struct SoftBodyProperties {
    real density = 1000;
    real young_modulus = 1e8;
    real poisson_ratio = 0.4999;

    real calc_mu() {
        return young_modulus / (1.0 + poisson_ratio);
    }

    real calc_lambda() {
        return young_modulus * poisson_ratio / ((1.0 + poisson_ratio) * (1.0 - 2.0 * poisson_ratio));
    }

    real calc_arap_stiffness() {
        return 2*calc_mu();
    }

    real calc_corotational_stiffness() {
        return 2*calc_mu() + calc_lambda();
    }

    real calc_neohookean_stiffness(real cmin = 0.9, real cmax = 1.1) {
        real mu = calc_mu();
        real lambda = calc_lambda();
        auto numer = [mu, lambda](real x) {
            real lgx = log(1+x);
            return mu*(lgx - x + x*x/2 + x*x*x/3) - lambda*((1+x)*(1-lgx) + lgx*lgx/2);
        };
        auto denom = [](real x) { return x*x*x/3; };
        return (numer(cmax-1) - numer(cmin-1)) / (denom(cmax-1) - denom(cmin-1));
    }
};

struct SoftBody {
    std::vector<glm::tvec3<real>> verts;
    std::vector<glm::ivec4> tets;

    std::vector<int> surface_verts;
    std::vector<glm::ivec2> surface_edges;
    std::vector<glm::ivec3> surface_triangles;

    std::vector<glm::tmat3x3<real>> B_m;
    std::vector<real> W;
    std::vector<glm::tmat4x3<real>> D;

    Eigen::SparseMatrix<real> M;
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<real>> M_LDLt;
    Eigen::SparseMatrix<real> A;
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<real>> A_LDLt;

    virtual void load(const TetMesh& mesh);
    virtual void load(const PyMesh::MshLoader& msh);

    void build_mass(real density);

    template <class Constraint>
    void add_volume_constraint(const Constraint& c, real dt = 1) {
        glm::ivec4 tet = tets[c.tet_id];
        for (int j = 0; j < 4; j++) {
            for (int k = 0; k < 4; k++) {
                real dL = c.k * (dt * dt) * W[c.tet_id] * glm::dot(D[c.tet_id][j], D[c.tet_id][k]);
                A.coeffRef(3*tet[j]+0, 3*tet[k]+0) += dL;
                A.coeffRef(3*tet[j]+1, 3*tet[k]+1) += dL;
                A.coeffRef(3*tet[j]+2, 3*tet[k]+2) += dL;
            }
        }
    }

    template <class Constraint>
    void add_positional_constraint(const Constraint& c, real dt = 1) {
        A.coeffRef(3*c.vert_id+0, 3*c.vert_id+0) += c.k * (dt * dt);
        A.coeffRef(3*c.vert_id+1, 3*c.vert_id+1) += c.k * (dt * dt);
        A.coeffRef(3*c.vert_id+2, 3*c.vert_id+2) += c.k * (dt * dt);
    }

    void factorize() {
        // Eigen::SimplicialLDLT<Eigen::SparseMatrix<real>> A_LDLt;
        A_LDLt.analyzePattern(A);
        A_LDLt.factorize(A);
        M_LDLt.analyzePattern(M);
        M_LDLt.factorize(M);
        /*
        fac_L = A_LDLt.matrixL();
        fac_D = A_LDLt.vectorD();
        fac_P = A_LDLt.permutationP();
         */
    }

    void clear_mass();
};

}

#endif //ARTSIM_SOFT_BODY_H
