//
// Created by lasagnaphil on 21. 7. 20..
//

#ifndef EOS_SCAN_TO_HUMAN_SOFT_BODY_DYNAMICS_H
#define EOS_SCAN_TO_HUMAN_SOFT_BODY_DYNAMICS_H

#include <artsim/soft_body.h>
#include <artsim/artsim.h>
#include <artsim/math/dynmat.h>
#include <artsim/math/svd.h>

namespace artsim {

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

struct PositionalEmbedConstraint {
    int tet_id;
    real k;
    glm::rvec3 betas;
    glm::rvec3 target_pos;
};

struct CollisionConstraint {
    int vert_id;
    real k;
    glm::rvec3 target_pos;
    glm::rvec3 normal;
};

struct SoftRigidCollisionEmbedConstraint {
    int tet_id;
    real k;
    glm::rvec3 betas;
    glm::rvec3 closest_point;
    glm::rvec3 normal;
};

struct SoftSoftCollisionConstraint {
    int vert_id;
    glm::ivec3 tri;
    real k;
    glm::rvec3 betas;
    glm::rvec3 normal;
};

struct PDConstraints {
    std::vector<LinearStrainEnergyConstraint> linear_strain_energy;
    std::vector<VolumePreservationEnergyConstraint> volume_preservation_energy;
    std::vector<PositionalConstraint> positional;
    std::vector<CollisionConstraint> soft_rigid_collision;

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

void soft_body_calc_deformation_field(const SoftBody& body, const glm::rvec3* x,
                                      OUT glm::rmat3* F);
void soft_body_calc_deformation_field(const SoftBody& body, const glm::rvec3* x, const glm::rmat3* u,
                                      OUT glm::rmat3* F);

template <class Constraint>
void projective_dynamics_volume_constraint_local_solve(
        const SoftBody& body, const Constraint* constraints, uint32_t num_constraints,
        const glmx::SVD_mats<real>* F_svd, OUT glm::tmat3x3<real>* p);

template <class Constraint>
void projective_dynamics_volume_constraint_update_b(
        const SoftBody& body, const Constraint* constraints, uint32_t num_constraints, real dt,
        const glm::tmat3x3<real>* p, INOUT real* b);

void projective_dynamics_positional_constraint_update_b(
        const SoftBody& body, const PositionalConstraint* constraints, uint32_t num_constraints, real dt,
        INOUT real* b);

template <class Constraint>
void admm_volume_constraint_local_solve(
        const SoftBody& body, const Constraint* constraints, uint32_t num_constraints,
        const glm::tmat3x3<real>* F, const glmx::SVD_mats<real>* F_svd,
        OUT glm::tmat3x3<real>* z, OUT glm::tmat3x3<real>* u);

template <class Constraint>
void admm_volume_constraint_update_b(
        const SoftBody& body, const Constraint* constraints, uint32_t num_constraints, real dt,
        const glm::tmat3x3<real>* z, const glm::tmat3x3<real>* u, const glm::tvec3<real>* x0, INOUT real* b);

template <class Constraint>
void admm_volume_constraint_update_residuals(
        const SoftBody& body, const Constraint* constraints, uint32_t num_constraints,
        const glm::tmat3x3<real>* z_prev, const glm::tmat3x3<real>* z_next, const glm::tvec3<real>* x,
        INOUT real& primal_res_sq, INOUT real& dual_res_sq);

void projective_dynamics(const SoftBody& body,
                         const PDConstraints& constraints, real dt, int num_iters, const real* f,
                         INOUT real* pos, INOUT real* vel);

void projective_dynamics_quasistatic(const SoftBody& body,
                                     const PDConstraints& constraints, int num_iters, const real* f,
                                     INOUT real* pos);

void admm_dynamics(const SoftBody& body,
                   const ADMMConstraints& constraints, real dt, int num_iters, const real* f,
                   INOUT real* pos, INOUT real* vel);

void quasinewton_dynamics(const SoftBody& body,
                          const ADMMConstraints& constraints, real dt, int num_iters, const real* f,
                          INOUT real* pos, INOUT real* vel);

}

#endif //EOS_SCAN_TO_HUMAN_SOFT_BODY_DYNAMICS_H
