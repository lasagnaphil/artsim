//
// Created by lasagnaphil on 2/16/21.
//

#ifndef ARTSIM_SOFT_BODY_WITH_ART_H
#define ARTSIM_SOFT_BODY_WITH_ART_H

#include <artsim/soft_body.h>

namespace artsim {

struct SoftBodyWithArtData {
    std::vector<glm::tvec3<real>> vertices;
    std::vector<glm::ivec3> triangles;
    std::vector<glm::ivec4> tetrahedrons;

    std::vector<glm::tmat3x3<real>> B_m;
    std::vector<real> W;
    std::vector<glm::tmat4x3<real>> D;

    Eigen::SparseMatrix<real> M;
    Eigen::SparseMatrix<real> M_ff;
    Eigen::SparseMatrix<real> M_fc;
    Eigen::SparseMatrix<real> M_cc;
    Eigen::SparseMatrix<real> A_ff;

    Eigen::SimplicialLDLT<Eigen::SparseMatrix<real>> M_LDLt;
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<real>> A_ff_LDLt;

    SoftBodyProperties props;

    ArticulatedBody art;

    std::unordered_map<int, std::pair<int, int>> constrained_vertices_range;
    std::unordered_map<int, glmx::ttransform<real>> constrained_vertices_offset;
    int constrained_idx_start;

    int N_s, N_c, N_f, N_r;

    // TODO: what is the best value for k_c?
    real k_c = 0.5;

    void load(const char* metadata);
};

void soft_body_precomputation(SoftBodyWithArtData& body, const ADMMConstraints& constraints, real dt);

void admm_dynamics_with_art(const SoftBodyWithArtData& data, const ADMMConstraints& constraints,
                            real dt, const real* sb_f, const real* art_f,
                            INOUT real* sb_pos, INOUT real* sb_vel,
                            INOUT real* art_pos, INOUT real* art_vel);

}

#endif //ARTSIM_SOFT_BODY_WITH_ART_H
