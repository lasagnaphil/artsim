//
// Created by lasagnaphil on 2/16/21.
//

#ifndef ARTSIM_SOFT_BODY_WITH_ART_H
#define ARTSIM_SOFT_BODY_WITH_ART_H

#include <artsim/soft_body.h>

namespace artsim {

struct SoftBodyWithArtData {
    SoftBodyData sb;
    ArticulatedBody art;

    Eigen::SparseMatrix<real> A_inv;

    std::unordered_map<int, std::pair<int, int>> constrained_vertices_range;
    std::unordered_map<int, glmx::ttransform<real>> constrained_vertices_offset;
    int num_constrained_vertices;
    int constrained_idx_start;

    // TODO: what is the best value for k_c?
    real k_c = 0.2;

    void load(const char* metadata);
};

void soft_body_precomputation(SoftBodyWithArtData& body, const ADMMConstraints& constraints, real dt);

void admm_dynamics_with_art(const SoftBodyWithArtData& data, const ADMMConstraints& constraints,
                            real dt, const real* sb_f, const real* art_f,
                            INOUT real* sb_pos, INOUT real* sb_vel,
                            INOUT real* art_pos, INOUT real* art_vel);

}

#endif //ARTSIM_SOFT_BODY_WITH_ART_H
