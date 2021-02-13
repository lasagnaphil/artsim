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

struct SoftBodyProperties {
    double density = 1000;
    double young_modulus = 1e8;
    double poisson_ratio = 0.4999;
    double stiffness = 10000;
    double dt = 1.0 / 60.0f;

    double calc_mu() {
        return young_modulus / (1.0 + poisson_ratio);
    }

    double calc_lambda() {
        return young_modulus * poisson_ratio / ((1.0 + poisson_ratio) * (1.0 - 2.0 * poisson_ratio));
    }
};

struct CorotationalEnergyConstraint {
    int tet_id;
    double mu;
    double lambda;
    double k;
};

struct VolumePreservationEnergyConstraint {
    int tet_id;
    double k;
    double sigma_min;
    double sigma_max;
};

struct SoftBodyData {
    std::vector<glm::dvec3> vertices;
    std::vector<glm::ivec3> triangles;
    std::vector<glm::ivec4> tetrahedrons;

    std::vector<glm::dmat3x3> B_m;
    std::vector<double> W;
    Eigen::SparseMatrix<double> M;
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> M_LDLt;
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> A_LDLt;

    SoftBodyProperties props;

    std::vector<glm::dmat4x3> D;

    std::vector<CorotationalEnergyConstraint> corotational_energy_constraints;
    std::vector<VolumePreservationEnergyConstraint> volume_preservation_energy_constraints;

    void load(const OBJFile& obj, const SoftBodyProperties& props);

    void precomputation();

    void add_corotational_energy(int tet_id, double k, double mu, double lambda);
    void add_corotational_energy_full_body(double k, double mu, double lambda);

    void add_volume_preservation_energy(int tet_id, double k, double sigma_min, double sigma_max);
    void add_volume_preservation_energy_full_body(double k, double sigma_min, double sigma_max);
};

enum class FEMAlgorithmType {
    ProjectiveDynamics,
    ADMM
};

void soft_body_dynamics(const SoftBodyData& body, FEMAlgorithmType alg_type, double dt, const double* f,
                        OUT double* pos, OUT double* vel);

}

#endif //ARTSIM_SOFT_BODY_H
