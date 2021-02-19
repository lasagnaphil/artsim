//
// Created by lasagnaphil on 2/16/21.
//

#include "artsim/soft_body_with_art.h"
#include "artsim/dynamics.h"
#include "artsim/utils/xml.h"

#include <glm/gtx/hash.hpp>
#include <filesystem>

using namespace glmx;
using namespace artsim;

SoftBodyWithArtData SoftBodyWithArtData::make_two_link_test() {
    SoftBodyWithArtData data;
    std::vector<uint32_t> contact_indices;
    data.art = load_from_xml("demo/resources/soft_body_with_art/two_link_art.xml", contact_indices);

}

void SoftBodyWithArtData::load(const OBJFile& soft_body_obj, const SoftBodyProperties& soft_body_props,
                                       const ArticulatedBody& in_art, const real* rest_pose_data) {
    soft_body.load(soft_body_obj, soft_body_props);
    this->art = in_art;

    int num_pos_dofs = art.get_num_pos_dofs();
    rest_pose.resize(num_pos_dofs);
    std::copy_n(rest_pose_data, num_pos_dofs, rest_pose.data());
}

void SoftBodyWithArtData::precomputation() {
    using namespace Eigen;
    // TODO: reorder vertices so that constrained ones go last

    int num_links = art.get_num_joints();

    // Calculate vertex jacobians
    std::vector<ttransform<real>> link_trans(num_links), joint_trans(num_links);
    calc_transforms(art, rest_pose.data(), link_trans.data(), joint_trans.data());

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


void soft_body_dynamics_with_art(const SoftBodyWithArtData& data, FEMAlgorithmType alg_type,
                                         real dt, const real* f, real* pos, real* vel) {


}
