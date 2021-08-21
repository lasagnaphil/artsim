//
// Created by Phillip Chang on 2020/09/12.
//

#ifndef ARTSIM_DYNAMICS_H
#define ARTSIM_DYNAMICS_H

#include "artsim/artsim.h"
#include "artsim/math/se3.h"
#include "artsim/math/dynmat.h"

#include <queue>
#include <iostream>
#include <chrono>

#include <glm/gtc/type_ptr.hpp>

namespace artsim {

    using namespace glm;

    struct ReducedJacobian {
        std::vector<int> dofs;
        glmx::dynmat<real> J;
    };

    inline int get_screw_idx(JointType joint_type) {
        return (int)joint_type;
    }

    inline glmx::tscrew<real> get_joint_screw(JointType joint_type) {
        glmx::tscrew<real> V(glmx::IDENTITY);
        V[get_screw_idx(joint_type)] = real(1);
        return V;
    }

    void set_zero_pose(const ArticulatedBodySpec& art, OUT real* q);

    void calc_S(const ArticulatedBodySpec& art, const real*__restrict q, OUT glmx::tscrew<real>* S);

    glmx::ttransform<real> calc_Tinv(const Joint& joint, const Link& link, const real*__restrict q);

    glmx::tscrew<real> calc_v0(const Joint& joint, const real*__restrict u);

    void calc_body_jacobian(const ArticulatedBodySpec& art, uint32_t joint_idx, const glmx::ttransform<real>& offset,
                            const glmx::ttransform<real>* T_joint_global,
                            OUT glmx::tscrew<real>* J_s);

    void calc_linear_jacobian(const ArticulatedBodySpec& art, uint32_t joint_idx, const glmx::rtransform& offset,
                              const glmx::rtransform* T_joint_global,
                              OUT glmx::dynmat_view<real> Jc);

    void calc_linear_jacobian_transpose(const ArticulatedBodySpec& art, uint32_t joint_idx, const glmx::rtransform& offset,
                                            const glmx::rtransform* T_joint_global,
                                            OUT glmx::dynmat_view<real> Jc_T);

    void rne_inverse_dynamics(const ArticulatedBodySpec& art,
                              glm::tvec3<real> gravity, real dt,
                              const real*__restrict q, const real*__restrict u, const real*__restrict udot,
                              const glmx::tscrew<real>*__restrict f_ext,
                              OUT real*__restrict tau);

    void multiply_mass_matrix(const ArticulatedBodySpec& art, real dt,
                              const real* q, const real* x,
                              OUT real* M_x);

    void featherstone_forward_dynamics(const ArticulatedBodySpec& art,
                                       glm::tvec3<real> gravity, real dt,
                                       const glmx::tscrew<real>*__restrict f_ext,
                                       const real*__restrict q, const real*__restrict u, const real*__restrict tau,
                                       const real*__restrict q_target,
                                       OUT real*__restrict udot);

    void featherstone_forward_dynamics(const ArticulatedBodySpec& art,
                                       glm::tvec3<real> gravity, real dt,
                                       const glmx::tscrew<real>*__restrict f_ext, const glmx::tscrew<real>*__restrict f_c,
                                       const real*__restrict q, const real*__restrict u, const real*__restrict tau,
                                       const real*__restrict q_target,
                                       OUT real*__restrict udot);

    void multiply_inverse_mass_matrix(const ArticulatedBodySpec& art, real dt,
                                      const real* q, const real* x,
                                      OUT real* Minv_x);

    void multiply_inverse_mass_matrix(const ArticulatedBodySpec& art, real dt,
                                      const real* q, glmx::dynmat_view<real> X,
                                      OUT glmx::dynmat_view<real> Minv_X);

    void mass_matrix_using_rnea(const ArticulatedBodySpec& art, real dt, const real*__restrict q, OUT glmx::dynmat<real>& M);

    void all_forces(const ArticulatedBodySpec& art,
                    glm::tvec3<real> gravity, real dt,
                    const glmx::tscrew<real>*__restrict f_ext,
                    const real*__restrict q, const real*__restrict u,
                    OUT real* tau);

    void forward_dynamics_using_rnea(const ArticulatedBodySpec& art,
                                     glm::tvec3<real> gravity, real dt,
                                     const glmx::tscrew<real>*__restrict f_ext,
                                     const real*__restrict q, const real*__restrict u, const real*__restrict tau,
                                     OUT real*__restrict udot);

    void integrate_velocities(const ArticulatedBodySpec& art, real dt, const real*__restrict udot, INOUT real*__restrict u);
    void integrate_positions(const ArticulatedBodySpec& art, real dt, const real*__restrict u, INOUT real*__restrict q);

    void integrate_implicit_euler(const ArticulatedBodySpec& art,
                                  real dt, const real*__restrict udot,
                                  INOUT real*__restrict q, INOUT real*__restrict u);

    void integrate_second_order(const ArticulatedBodySpec& art, real dt, const real* udot,
                                INOUT real*__restrict q, INOUT real*__restrict u);

    void calc_transforms(const ArticulatedBodySpec& art, const real* q, glmx::ttransform<real>* T_joint_globals,
                         OUT glmx::ttransform<real>* T_link_globals);

    void calc_velocities(const ArticulatedBodySpec& art, const real* q, const real* u,
                         OUT glmx::rscrew* link_V);

    // Mass matrix calculation using the composite-rigid-body algorithm.
    void mass_matrix(const ArticulatedBodySpec& art, real dt, const real*__restrict q, OUT glmx::dynmat_view<real> M);
}

#endif //ARTSIM_DYNAMICS_H
