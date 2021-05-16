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

    void calc_body_jacobian(const ArticulatedBodySpec& art, uint32_t joint_idx, glmx::ttransform<real> offset,
                            const glmx::tscrew<real>* S,
                            const glmx::ttransform<real>* T_joint_global,
                            OUT glmx::tscrew<real>* J_s);

    void rne_inverse_dynamics(const ArticulatedBodySpec& art,
                              glm::tvec3<real> gravity, real dt,
                              const real*__restrict q, const real*__restrict u, const real*__restrict udot,
                              const glmx::tscrew<real>*__restrict f_ext,
                              OUT real*__restrict tau);

    void featherstone_forward_dynamics(const ArticulatedBodySpec& art,
                                       glm::tvec3<real> gravity, real dt,
                                       const glmx::tscrew<real>*__restrict f_ext,
                                       const real*__restrict q, const real*__restrict u, const real*__restrict tau,
                                       OUT real*__restrict udot);
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

    void integrate_implicit_euler(const ArticulatedBodySpec& art,
                                  real dt, const real*__restrict udot,
                                  OUT real*__restrict q, OUT real*__restrict u);

    void calc_transforms(const ArticulatedBodySpec& art, const real* q, glmx::ttransform<real>* T_joint_globals,
                         glmx::ttransform<real>* T_link_globals);

    // Mass matrix calculation using the composite-rigid-body algorithm.
    void mass_matrix(const ArticulatedBodySpec& art, real dt, const real*__restrict q, OUT glmx::dynmat_view<real> M);
}

#endif //ARTSIM_DYNAMICS_H
