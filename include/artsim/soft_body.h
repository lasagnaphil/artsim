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

struct LinearStrainEnergyConstraint {
    int tet_id;
    real k;
    real sigma_min;
    real sigma_max;
};

struct VolumePreservationEnergyConstraint {
    int tet_id;
    real k;
    real sigma_min;
    real sigma_max;
};

struct CorotationalEnergyConstraint {
    int tet_id;
    real k;
    real mu;
    real lambda;
};

struct NeoHookeanEnergyConstraint {
    int tet_id;
    real k;
    real mu;
    real lambda;
};

struct PDConstraints {
    std::vector<LinearStrainEnergyConstraint> linear_strain_energy;
    std::vector<VolumePreservationEnergyConstraint> volume_preservation_energy;
};

#define PD_VOLUME_CONSTRAINTS \
    X(LinearStrainEnergyConstraint, linear_strain_energy) \
    X(VolumePreservationEnergyConstraint, volume_preservation_energy)

struct ADMMConstraints {
    std::vector<CorotationalEnergyConstraint> corotational_energy;
    std::vector<NeoHookeanEnergyConstraint> neohookean_energy;
};

#define ADMM_VOLUME_CONSTRAINTS \
    X(CorotationalEnergyConstraint, corotational_energy) \
    X(NeoHookeanEnergyConstraint, neohookean_energy)

void gen_surface_triangles_from_tet_mesh(const std::vector<glm::ivec4>& tetrahedrons,
                                         OUT std::vector<glm::ivec3>& triangles);

struct SoftBodyData {
    std::vector<glm::tvec3<real>> vertices;
    std::vector<glm::ivec3> triangles;
    std::vector<glm::ivec4> tetrahedrons;

    std::vector<glm::tmat3x3<real>> B_m;
    std::vector<real> W;
    std::vector<glm::tmat4x3<real>> D;

    Eigen::SparseMatrix<real> M;
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<real>> M_LDLt;
    Eigen::SparseMatrix<real> A;
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<real>> A_LDLt;

    SoftBodyProperties props;

    void load(const TetMesh& mesh, const SoftBodyProperties& props);
    void load(const PyMesh::MshLoader& msh, const SoftBodyProperties& props);
};

void precomputation_essentials(SoftBodyData& body);

template <class Constraints>
void soft_body_precomputation(SoftBodyData& body, const Constraints& constraints, real dt);

void tetrahedral_mesh_mass_matrix(int num_vertices, real density,
                                  const glm::ivec4* tets, int num_tets,
                                  const real* tet_volumes,
                                  OUT Eigen::SparseMatrix<real>& M);

template <class Constraints>
void update_system_matrix(SoftBodyData& body, const Constraints& constraints, real dt, OUT Eigen::SparseMatrix<real>& A);

glm::tmat3x3<real> projection(const glm::tmat3x3<real>& F, const LinearStrainEnergyConstraint& c);
glm::tmat3x3<real> projection(const glm::tmat3x3<real>& F, const VolumePreservationEnergyConstraint& c);

glm::tvec3<real> proximal_eigvec(glm::tvec3<real> sigma, const CorotationalEnergyConstraint& c);
glm::tvec3<real> proximal_eigvec(glm::tvec3<real> sigma, const NeoHookeanEnergyConstraint& c);

glm::tmat3x3<real> proximal(const glm::tmat3x3<real>& F, const CorotationalEnergyConstraint& c);
glm::tmat3x3<real> proximal(const glm::tmat3x3<real>& F, const NeoHookeanEnergyConstraint& c);

template <class Constraint>
void projective_dynamics_volume_constraint_local_solve(
        const SoftBodyData& body, const Constraint* constraints, uint32_t num_constraints,
        const glm::tvec3<real>* V,
        OUT glm::tmat3x3<real>* p);

template <class Constraint>
void admm_volume_constraint_local_solve(
        const SoftBodyData& body, const Constraint* constraints, uint32_t num_constraints,
        const glm::tvec3<real>* V,
        OUT glm::tmat3x3<real>* z, OUT glm::tmat3x3<real>* u,
        OUT glm::tmat3x3<real>* F, OUT glmx::SVD_mats<real>* F_svd);

template <class Constraint>
void admm_volume_constraint_update_b(
        const SoftBodyData& body, const Constraint* constraints, uint32_t num_constraints, real dt,
        const glm::tmat3x3<real>* z, const glm::tmat3x3<real>* u, const glm::tvec3<real>* x0, INOUT real* b);

template <class Constraint>
void admm_volume_constraint_update_residuals(
        const SoftBodyData& body, const Constraint* constraints, uint32_t num_constraints,
        const glm::tmat3x3<real>* z_prev, const glm::tmat3x3<real>* z_next, const glm::tvec3<real>* x,
        INOUT real& primal_res_sq, INOUT real& dual_res_sq);

void admm_dynamics(SoftBodyData& body, const ADMMConstraints& constraints, real dt, const real* f,
                   INOUT real* pos, INOUT real* vel);
}

#endif //ARTSIM_SOFT_BODY_H
