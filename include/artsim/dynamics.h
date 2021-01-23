//
// Created by Phillip Chang on 2020/09/12.
//

#ifndef ARTSIM_DYNAMICS_H
#define ARTSIM_DYNAMICS_H

#include "artsim/artsim.h"
#include "artsim/math/se3.h"
#include "artsim/math/dynmat.h"

#include "artsim/collision.h"

#include <queue>
#include <iostream>
#include <chrono>

#include <Eigen/Dense>
#include <glm/gtc/type_ptr.hpp>

// #define SHOW_LOG

#ifdef SHOW_LOG
#define output_log(...) printf(__VA_ARGS__)
#else
#define output_log(...)
#endif

namespace artsim {

    using namespace glm;

    inline int get_screw_idx(JointType joint_type) {
        return (int)joint_type;
    }

    inline tscrew<real> get_joint_screw(JointType joint_type) {
        tscrew<real> V;
        V[get_screw_idx(joint_type)] = real(1);
        return V;
    }

    void calc_S(const ArticulatedBody& art, const real*__restrict q, OUT tscrew<real>* S);

    ttransform<real> calc_Tinv(const Joint& joint, const Link& link, const real*__restrict q);

    tscrew<real> calc_v0(const Joint& joint, const real*__restrict u);

    void calculate_jacobian_for_local_frame(const ArticulatedBody& art,
                                      uint32_t link_idx,
                                      const ttransform<real>& T_contact,
                                      const ttransform<real>*__restrict T_joint_global,
                                      const tscrew<real>*__restrict S,
                                      OUT tscrew<real>* J_local);

    void rne_inverse_dynamics(const ArticulatedBody& art,
                              const real*__restrict q, const real*__restrict u, const real*__restrict udot,
                              glm::tvec3<real> gravity,
                              const tscrew<real>*__restrict f_ext,
                              OUT real*__restrict tau);

    void featherstone_forward_dynamics(const ArticulatedBody& art,
                                       glm::tvec3<real> gravity,
                                       const tscrew<real>*__restrict f_ext,
                                       const real*__restrict q, const real*__restrict u, const real*__restrict tau,
                                       OUT real*__restrict udot);

    enum class ContactSolverType {
        PGS, Bisection, NCP
    };

    void solve_collision(ContactSolverType type,
                         const ArticulatedBody& art,
                         const MaterialDB& material_db,
                         glm::tvec3<real> gravity, real dt,
                         const real*__restrict q, const real*__restrict u, const real*__restrict udot_orig,
                         const tscrew<real>*__restrict f_ext, const real* tau,
                         const ContactPoint*__restrict contact_points, uint32_t num_contact_points,
                         OUT glm::tvec3<real>*__restrict out_lambda, OUT real*__restrict out_contact_forces);

    void mass_matrix_using_rnea(const ArticulatedBody& art, const real*__restrict q, OUT real*__restrict M);

    void all_forces(const ArticulatedBody& art,
                    glm::tvec3<real> gravity,
                    const tscrew<real>*__restrict f_ext,
                    const real*__restrict q, const real*__restrict u,
                    OUT real* tau);

    void forward_dynamics_using_rnea(const ArticulatedBody& art,
                                     glm::tvec3<real> gravity,
                                     const tscrew<real>*__restrict f_ext,
                                     const real*__restrict q, const real*__restrict u, const real*__restrict tau,
                                     OUT real*__restrict udot);

    void integrate_implicit_euler(const ArticulatedBody& art,
                                  real dt, const real*__restrict udot,
                                  OUT real*__restrict q, OUT real*__restrict u);

    void calc_transforms(const ArticulatedBody& art,
                         const real*__restrict q,
                         OUT ttransform<real>* T_link_globals,
                         OUT ttransform<real>* T_joint_globals);

    // Mass matrix calculation using the composite-rigid-body algorithm.
    void mass_matrix(const ArticulatedBody& art, const real*__restrict q, OUT real*__restrict M_ptr);

    void euler_step_with_collision(ContactSolverType type,
                                   const ArticulatedBody& art,
                                   const MaterialDB& material_db,
                                   glm::tvec3<real> gravity, real dt,
                                   const tscrew<real>*__restrict f_ext,
                                   const real*__restrict tau,
                                   const ContactPoint* contact_points, uint32_t num_contact_points,
                                   INOUT real*__restrict q, INOUT real*__restrict u,
                                   OUT real*__restrict udot, OUT glm::tvec3<real>* lambda);

}

#endif //ARTSIM_DYNAMICS_H
