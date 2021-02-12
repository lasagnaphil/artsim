//
// Created by lasagnaphil on 21. 2. 8..
//

#ifndef ARTSIM_SOFT_BODY_H
#define ARTSIM_SOFT_BODY_H

#include <glm/vec3.hpp>
#include <vector>
#include <artsim/artsim.h>
#include <artsim/math/dynmat.h>
#include <Eigen/SparseCholesky>

namespace artsim {

struct OBJFile {
    std::vector<glm::dvec3> vertices;
    std::vector<glm::dvec3> normals;
    std::vector<glm::dvec2> uvs;

    std::vector<glm::ivec3> triangle_vertices;
    std::vector<glm::ivec3> triangle_normals;
    std::vector<glm::ivec3> triangle_uvs;

    std::vector<glm::ivec4> tetrahedrons;

    void load(const char* filename);
};

struct SoftBodyData {
    std::vector<glm::dvec3> vertices;
    std::vector<glm::ivec3> triangles;
    std::vector<glm::ivec4> tetrahedrons;

    std::vector<glm::dmat3x3> B_m;
    std::vector<double> W;
    Eigen::SparseMatrix<double> M;
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> A_LDLt;

    double density = 1000;
    double stiffness = 1e6;
    double dt;

    void load(const OBJFile& obj);

    void precomputation(double dt);
};

enum class FEMAlgorithmType {
    ProjectiveDynamics,
    ADMM
};

void soft_body_dynamics(const SoftBodyData& body, FEMAlgorithmType alg_type, double dt,
                        OUT double* pos, OUT double* vel);

}

#endif //ARTSIM_SOFT_BODY_H
