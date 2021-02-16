//
// Created by lasagnaphil on 2/16/21.
//

#ifndef ARTSIM_SOFT_BODY_WITH_ART_H
#define ARTSIM_SOFT_BODY_WITH_ART_H

#include <artsim/soft_body.h>

namespace artsim {

struct SoftBodyWithArtData {
    SoftBodyData soft_body;

    std::unordered_map<int, std::pair<int, int>> constrained_vertices;
    int num_constrained_vertices;

    Eigen::SparseMatrix<real> M_ff;
    Eigen::SparseMatrix<real> M_cf;
    Eigen::SparseMatrix<real> J_M_fc;
    Eigen::SparseMatrix<real> J_M_cc;

    Eigen::SimplicialLDLT<Eigen::SparseMatrix<real>> M_LDLt;
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<real>> A_LDLt;

    ArticulatedBody art;
    std::vector<real> rest_pose;

    static SoftBodyWithArtData make_two_link_test();

    void load(const OBJFile& soft_body, const SoftBodyProperties& soft_body_props, const ArticulatedBody& art,
              const real* rest_pose_data);

    void precomputation();
};

void soft_body_dynamics_with_art(const SoftBodyWithArtData& data, FEMAlgorithmType alg_type, real dt, const real* f,
                                 OUT real* pos, OUT real* vel);

}

#endif //ARTSIM_SOFT_BODY_WITH_ART_H
