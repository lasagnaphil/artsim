//
// Created by lasagnaphil on 2/16/21.
//

#ifndef ARTSIM_SOFT_BODY_WITH_ART_H
#define ARTSIM_SOFT_BODY_WITH_ART_H

#include <artsim/soft_body.h>

namespace artsim {

struct SoftBodyWithArtData : public SoftBodyData {
public:
    ~SoftBodyWithArtData() = default;

    std::unordered_map<int, std::pair<int, int>> constrained_vertices;
    int num_constrained_vertices;

    std::vector<glm::tmat3x3<real>> B_m;
    std::vector<real> W;
    Eigen::SparseMatrix<real> M;
    std::vector<glm::tmat4x3<real>> D;

    Eigen::SparseMatrix<real> M_ff;
    Eigen::SparseMatrix<real> M_cf;
    Eigen::SparseMatrix<real> J_M_fc;
    Eigen::SparseMatrix<real> J_M_cc;

    ArticulatedBody art;
    std::vector<real> rest_pose;

    void load(const char* metadata);

    void precomputation();
};

void soft_body_dynamics_with_art(const SoftBodyWithArtData& data, FEMAlgorithmType alg_type, real dt, const real* f,
                                 OUT real* pos, OUT real* vel);

}

#endif //ARTSIM_SOFT_BODY_WITH_ART_H
