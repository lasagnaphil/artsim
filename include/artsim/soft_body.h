//
// Created by lasagnaphil on 21. 2. 8..
//

#ifndef ARTSIM_SOFT_BODY_H
#define ARTSIM_SOFT_BODY_H

#include <glm/vec3.hpp>
#include <vector>
#include <artsim/artsim.h>
#include <artsim/obj_file.h>
#include <artsim/math/dynmat.h>
#include <artsim/utils/pymesh/MshLoader.h>
#include <Eigen/SparseCholesky>

namespace artsim {

struct SoftBodyProperties {
    real density = 1000;
    real young_modulus = 1e8;
    real poisson_ratio = 0.4999;
    real dt = 1.0 / 60.0f;

    real calc_mu() {
        return young_modulus / (1.0 + poisson_ratio);
    }

    real calc_lambda() {
        return young_modulus * poisson_ratio / ((1.0 + poisson_ratio) * (1.0 - 2.0 * poisson_ratio));
    }
};

struct CorotationalEnergyConstraint {
    int tet_id;
    real k;
    real mu;
    real lambda;
};

struct VolumePreservationEnergyConstraint {
    int tet_id;
    real k;
    real sigma_min;
    real sigma_max;
};

struct SoftBodyData {
    std::vector<glm::tvec3<real>> vertices;
    std::vector<glm::ivec3> triangles;
    std::vector<glm::ivec4> tetrahedrons;

    std::vector<glm::tmat3x3<real>> B_m;
    std::vector<real> W;
    Eigen::SparseMatrix<real> M;
    std::vector<glm::tmat4x3<real>> D;

    Eigen::SimplicialLDLT<Eigen::SparseMatrix<real>> M_LDLt;
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<real>> A_LDLt;

    SoftBodyProperties props;

    std::vector<CorotationalEnergyConstraint> corotational_energy_constraints;
    std::vector<VolumePreservationEnergyConstraint> volume_preservation_energy_constraints;

    void load(const OBJFile& obj, const SoftBodyProperties& props);
    void load(const PyMesh::MshLoader& msh, const SoftBodyProperties& props);

    void precomputation();

    void add_corotational_energy(int tet_id, real k, real mu, real lambda);
    void add_corotational_energy_full_body(real k, real mu, real lambda);

    void add_volume_preservation_energy(int tet_id, real k, real sigma_min, real sigma_max);
    void add_volume_preservation_energy_full_body(real k, real sigma_min, real sigma_max);

private:
    void generate_surface_triangles();
};

enum class FEMAlgorithmType {
    ProjectiveDynamics,
    ADMM
};

void soft_body_dynamics(const SoftBodyData& body, FEMAlgorithmType alg_type, real dt, const real* f,
                        OUT real* pos, OUT real* vel);

}

#endif //ARTSIM_SOFT_BODY_H
