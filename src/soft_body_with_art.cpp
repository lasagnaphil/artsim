//
// Created by lasagnaphil on 2/16/21.
//

#include "artsim/soft_body_with_art.h"
#include "artsim/dynamics.h"

#include <glm/gtx/hash.hpp>
#include <filesystem>

using namespace glmx;

glm::ivec3 reorder_tri_indices(glm::ivec3 tri) {
    while (tri[0] > tri[1] || tri[0] > tri[2]) {
        std::swap(tri[0], tri[1]);
        std::swap(tri[1], tri[2]);
    }
    return tri;
}

void artsim::SoftBodyWithArtData::load(const artsim::OBJFile& soft_body_obj, const artsim::SoftBodyProperties& soft_body_props,
                                       const artsim::ArticulatedBody& in_art, const real* rest_pose_data) {
    soft_body.load(soft_body_obj, soft_body_props);
    this->art = in_art;

    int num_pos_dofs = art.get_num_pos_dofs();
    rest_pose.resize(num_pos_dofs);
    std::copy_n(rest_pose_data, num_pos_dofs, rest_pose.data());
}

void artsim::SoftBodyWithArtData::precomputation() {
    using namespace Eigen;
    // TODO: reorder vertices so that constrained ones go last

    int num_links = art.get_num_joints();

    // Calculate vertex jacobians
    std::vector<ttransform<real>> link_trans(num_links), joint_trans(num_links);
    artsim::calc_transforms(art, rest_pose.data(), link_trans.data(), joint_trans.data());

    std::vector<tscrew<real>> global_joint_S(num_links);
    calc_S(art, rest_pose.data(), global_joint_S.data());
    for (int link_idx = 0; link_idx < num_links; link_idx++) {
        global_joint_S[link_idx] = Ad(joint_trans[link_idx], global_joint_S[link_idx]);
    }

    Matrix<real, Dynamic, Dynamic> J_cr(3*num_constrained_vertices, num_links);

    for (auto& [link_idx, vertices] : constrained_vertices) {
        for (int vidx = vertices.first; vidx < vertices.second; vidx++) {
            auto T_v = soft_body.vertices[vidx] - joint_trans[link_idx].v;
            auto vel = glm::cross(global_joint_S[link_idx].w, T_v);
            J_cr(3*vidx+0, link_idx) = vel[0];
            J_cr(3*vidx+1, link_idx) = vel[1];
            J_cr(3*vidx+2, link_idx) = vel[2];
        }
    }

    // TODO
}

void artsim::soft_body_dynamics_with_art(const artsim::SoftBodyWithArtData& data, artsim::FEMAlgorithmType alg_type,
                                         artsim::real dt, const artsim::real* f, artsim::real* pos, artsim::real* vel) {


}
