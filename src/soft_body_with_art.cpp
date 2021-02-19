//
// Created by lasagnaphil on 2/16/21.
//

#include "artsim/soft_body_with_art.h"
#include "artsim/dynamics.h"
#include "artsim/utils/xml.h"

#include <tinyxml2.h>
#include <glm/gtx/hash.hpp>

using namespace glmx;
using namespace artsim;
using namespace tinyxml2;

void SoftBodyWithArtData::load(const char* metadata) {
    XMLDocument doc;
    doc.LoadFile(metadata);
    auto root_el = doc.RootElement();
    auto constraints_el = root_el->FirstChildElement("constraints");
    auto articulation_el = root_el->FirstChildElement("articulation");
    auto soft_body_mesh_el = root_el->FirstChildElement("soft_body_mesh");

    OBJFile soft_body_obj;
    soft_body_obj.load(soft_body_mesh_el->Attribute("file"));
    props.young_modulus = soft_body_mesh_el->DoubleAttribute("young_modulus");
    props.poisson_ratio = soft_body_mesh_el->DoubleAttribute("poisson_ratio");
    props.dt = 1.0 / soft_body_mesh_el->IntAttribute("hz");

    vertices = soft_body_obj.vertices;
    tetrahedrons = soft_body_obj.tetrahedrons;

    std::vector<uint32_t> contact_indices;
    art = load_from_xml(articulation_el->Attribute("file"), contact_indices);

    int num_pos_dofs = art.get_num_pos_dofs();
    rest_pose.resize(num_pos_dofs, 0);

    // TODO: reorder vertices so that constrained ones go last

    gen_surface_triangles_from_tet_mesh(tetrahedrons, triangles);
}

void SoftBodyWithArtData::precomputation() {
    using namespace Eigen;

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

    for (auto& [link_idx, link_vertices] : constrained_vertices) {
        for (int vidx = link_vertices.first; vidx < link_vertices.second; vidx++) {
            auto T_v = vertices[vidx] - joint_trans[link_idx].v;
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
