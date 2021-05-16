//
// Created by lasagnaphil on 3/1/21.
//

#ifndef ARTSIM_IK_H
#define ARTSIM_IK_H

#include <artsim/art_dynamics.h>

namespace artsim {

void inverse_kinematics(const ArticulatedBodySpec& art, uint32_t ee_idx, const glmx::ttransform<real>& ee_offset,
                        const glm::tvec3<real> ee_global_pos, int num_iters, INOUT real* q);

}

#endif //ARTSIM_IK_H
