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

struct ARAPEnergyConstraint {
    int tet_id;
    real k;
    real mu;
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

struct PositionalConstraint {
    int vert_id;
    real k;
    glm::rvec3 target_pos;
};

struct SoftRigidCollisionConstraint {
    int vert_id;
    real k;
    glm::rvec3 closest_point;
    glm::rvec3 normal;
};


struct PDConstraints {
    std::vector<LinearStrainEnergyConstraint> linear_strain_energy;
    std::vector<VolumePreservationEnergyConstraint> volume_preservation_energy;
    std::vector<PositionalConstraint> positional;
    std::vector<SoftRigidCollisionConstraint> soft_rigid_collision;

    int count() {
        return linear_strain_energy.size() + volume_preservation_energy.size() + positional.size()
            + soft_rigid_collision.size();
    }
};

#define PD_VOLUME_CONSTRAINTS \
    X(LinearStrainEnergyConstraint, linear_strain_energy) \
    X(VolumePreservationEnergyConstraint, volume_preservation_energy)

struct ADMMConstraints {
    std::vector<ARAPEnergyConstraint> arap_energy;
    std::vector<CorotationalEnergyConstraint> corotational_energy;
    std::vector<NeoHookeanEnergyConstraint> neohookean_energy;
    std::vector<PositionalConstraint> positional;
};

#define ADMM_VOLUME_CONSTRAINTS \
    X(ARAPEnergyConstraint, arap_energy) \
    X(CorotationalEnergyConstraint, corotational_energy) \
    X(NeoHookeanEnergyConstraint, neohookean_energy)

void gen_surface_triangles_from_tet_mesh(const std::vector<glm::ivec4>& tetrahedrons,
                                         OUT std::vector<glm::ivec3>& triangles);

struct SoftBodyPrecalcData {
    Eigen::SparseMatrix<real> A;
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<real>> A_LDLt;
    Eigen::SparseMatrix<real> L;
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<real>> L_LDLt;
};

struct SoftBodyData {
    std::vector<glm::tvec3<real>> vertices;
    std::vector<glm::ivec4> tetrahedrons;

    std::vector<glm::ivec2> surface_edges;
    std::vector<glm::ivec3> surface_triangles;

    std::vector<glm::tmat3x3<real>> B_m;
    std::vector<real> W;
    std::vector<glm::tmat4x3<real>> D;

    Eigen::SparseMatrix<real> M;
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<real>> M_LDLt;

    SoftBodyProperties props;

    void load(const TetMesh& mesh, const SoftBodyProperties& props);
    void load(const PyMesh::MshLoader& msh, const SoftBodyProperties& props);

    btTriangleIndexVertexArray create_bullet_surface_trimesh();
};

template <class Constraints>
void soft_body_precomputation(SoftBodyData& body, const Constraints& constraints, real dt, OUT SoftBodyPrecalcData& precalc);

void tetrahedral_mesh_mass_matrix(int num_vertices, real density,
                                  const glm::ivec4* tets, int num_tets,
                                  const real* tet_volumes,
                                  OUT Eigen::SparseMatrix<real>& M);

template <class Constraints>
void update_system_matrix(SoftBodyData& body, const Constraints& constraints, real dt,
                          OUT Eigen::SparseMatrix<real>& L, OUT Eigen::SparseMatrix<real>& A);

real energy_eigvec(glm::tvec3<real> S, const ARAPEnergyConstraint& c);
real energy_eigvec(glm::tvec3<real> S, const CorotationalEnergyConstraint& c);
real energy_eigvec(glm::tvec3<real> S, const NeoHookeanEnergyConstraint& c);

glm::tmat3x3<real> projection(const glm::tmat3x3<real>& F, const ARAPEnergyConstraint& c);
glm::tmat3x3<real> projection(const glm::tmat3x3<real>& F, const LinearStrainEnergyConstraint& c);
glm::tmat3x3<real> projection(const glm::tmat3x3<real>& F, const VolumePreservationEnergyConstraint& c);

glm::tvec3<real> proximal_eigvec(glm::tvec3<real> sigma, real volume, const ARAPEnergyConstraint& c);
glm::tvec3<real> proximal_eigvec(glm::tvec3<real> sigma, real volume, const CorotationalEnergyConstraint& c);
glm::tvec3<real> proximal_eigvec(glm::tvec3<real> sigma, real volume, const NeoHookeanEnergyConstraint& c);

glm::tmat3x3<real> proximal(const glm::tmat3x3<real>& F, real volume, const ARAPEnergyConstraint& c);
glm::tmat3x3<real> proximal(const glm::tmat3x3<real>& F, real volume, const CorotationalEnergyConstraint& c);
glm::tmat3x3<real> proximal(const glm::tmat3x3<real>& F, real volume, const NeoHookeanEnergyConstraint& c);

void soft_body_calc_deformation_field(const SoftBodyData& body, const glm::rvec3* x,
                                      OUT glm::rmat3* F);
void soft_body_calc_deformation_field(const SoftBodyData& body, const glm::rvec3* x, const glm::rmat3* u,
                                      OUT glm::rmat3* F);

template <class Constraint>
void projective_dynamics_volume_constraint_local_solve(
        const SoftBodyData& body, const Constraint* constraints, uint32_t num_constraints,
        const glmx::SVD_mats<real>* F_svd, OUT glm::tmat3x3<real>* p);

template <class Constraint>
void projective_dynamics_volume_constraint_update_b(
        const SoftBodyData& body, const Constraint* constraints, uint32_t num_constraints, real dt,
        const glm::tmat3x3<real>* p, INOUT real* b);

void projective_dynamics_positional_constraint_update_b(
        const SoftBodyData& body, const PositionalConstraint* constraints, uint32_t num_constraints, real dt,
        INOUT real* b);

template <class Constraint>
void admm_volume_constraint_local_solve(
        const SoftBodyData& body, const Constraint* constraints, uint32_t num_constraints,
        const glm::tmat3x3<real>* F, const glmx::SVD_mats<real>* F_svd,
        OUT glm::tmat3x3<real>* z, OUT glm::tmat3x3<real>* u);

template <class Constraint>
void admm_volume_constraint_update_b(
        const SoftBodyData& body, const Constraint* constraints, uint32_t num_constraints, real dt,
        const glm::tmat3x3<real>* z, const glm::tmat3x3<real>* u, const glm::tvec3<real>* x0, INOUT real* b);

template <class Constraint>
void admm_volume_constraint_update_residuals(
        const SoftBodyData& body, const Constraint* constraints, uint32_t num_constraints,
        const glm::tmat3x3<real>* z_prev, const glm::tmat3x3<real>* z_next, const glm::tvec3<real>* x,
        INOUT real& primal_res_sq, INOUT real& dual_res_sq);

void projective_dynamics(const SoftBodyData& body, const SoftBodyPrecalcData& precalc,
                         const PDConstraints& constraints, real dt, int num_iters, const real* f,
                         INOUT real* pos, INOUT real* vel);

void projective_dynamics_quasistatic(const SoftBodyData& body, const SoftBodyPrecalcData& precalc,
                                     const PDConstraints& constraints, int num_iters, const real* f,
                                     INOUT real* pos);

void admm_dynamics(const SoftBodyData& body, const SoftBodyPrecalcData& precalc,
                       const ADMMConstraints& constraints, real dt, int num_iters, const real* f,
                       INOUT real* pos, INOUT real* vel);

void quasinewton_dynamics(const SoftBodyData& body, const SoftBodyPrecalcData& precalc,
                          const ADMMConstraints& constraints, real dt, int num_iters, const real* f,
                          INOUT real* pos, INOUT real* vel);

}

#endif //ARTSIM_SOFT_BODY_H
