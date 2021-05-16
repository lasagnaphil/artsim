//
// Created by lasagnaphil on 1/26/21.
//

#ifndef ARTSIM_ART_CONTACTS_H
#define ARTSIM_ART_CONTACTS_H

#include "artsim/artsim.h"
#include "artsim/math/dynmat.h"

#include <glm/vec3.hpp>

namespace artsim {

void solve_collision(
        ContactSolverType type, uint32_t max_iters,
        const ArticulatedBodySpec& art,
        const Material& mat,
        glm::tvec3 <real> gravity, real dt,
        const real* __restrict q, const real* __restrict u, const real* __restrict udot_orig,
        const glmx::tscrew<real>* __restrict f_ext, const real* tau,
        const ContactPoint* __restrict contact_points, uint32_t num_contact_points,
        OUT glm::tvec3 <real>* __restrict out_lambda, OUT real*__restrict out_contact_forces);

void iterative_contact_solver(
        ContactSolverType type, uint32_t max_iters, const Material& mat, real dt,
        uint32_t num_contact_points,
        const glmx::dynmat<glm::tmat3x3<real>>& M_contact_inv,
        INOUT glm::tvec3<real>* c, INOUT glm::tvec3<real>* lambda);


void euler_step_with_collision(
        ContactSolverType type, uint32_t max_iters,
        const ArticulatedBodySpec& art,
        const Material& mat,
        glm::tvec3<real> gravity, real dt,
        const glmx::tscrew<real>*__restrict f_ext,
        const real*__restrict tau,
        const ContactPoint* contact_points, uint32_t num_contact_points,
        INOUT real*__restrict q, INOUT real*__restrict u,
        OUT real*__restrict udot, OUT glm::tvec3<real>* lambda);

std::vector<ContactPoint> get_contact_points_bullet(btCollisionWorld* bt_world);

std::vector<ContactPoint> contact_points_between_art_links_and_ground(
        const ArticulatedBodySpec &art,
        const Id<ArticulatedBodySpec> art_id,
        const uint32_t* link_indices, uint32_t link_indices_count,
        const glmx::ttransform<real>* link_global_trans);

}

#endif //ARTSIM_ART_CONTACTS_H
