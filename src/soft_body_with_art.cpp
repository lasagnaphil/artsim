//
// Created by lasagnaphil on 2/16/21.
//

#include "artsim/soft_body_with_art.h"
#include "artsim/soft_body.h"
#include "artsim/dynamics.h"
#include "artsim/utils/xml.h"

#include <unordered_set>
#include <tinyxml2.h>
#include <glm/gtx/hash.hpp>
#include <filesystem>

using namespace glmx;
using namespace tinyxml2;
namespace fs = std::filesystem;

namespace artsim {

void SoftBodyWithArtData::load(const char* metadata) {

    fs::path metadata_path(metadata);
    fs::path folder = metadata_path.parent_path();

    XMLDocument doc;
    doc.LoadFile(metadata);
    auto root_el = doc.RootElement();
    auto constraints_el = root_el->FirstChildElement("constraints");
    auto articulation_el = root_el->FirstChildElement("articulation");
    auto soft_body_mesh_el = root_el->FirstChildElement("soft_body_mesh");

    OBJFile soft_body_obj;
    fs::path soft_body_file = folder / soft_body_mesh_el->Attribute("file");
    if (soft_body_file.extension() == ".obj") {
        soft_body_obj.load_obj(soft_body_file.c_str());
    }
    else if (soft_body_file.extension() == ".msh") {
        soft_body_obj.load_msh(soft_body_file.c_str());
    }
    else {
        fprintf(stderr, "Invalid extension name for soft body mesh!\n");
        exit(EXIT_FAILURE);
    }

    sb.props.young_modulus = soft_body_mesh_el->DoubleAttribute("young_modulus");
    sb.props.poisson_ratio = soft_body_mesh_el->DoubleAttribute("poisson_ratio");

    std::vector<uint32_t> contact_indices;
    auto art_file = folder / articulation_el->Attribute("file");
    art = load_from_xml(art_file.c_str(), contact_indices);

    // reorder vertices so that constrained ones go last
    std::map<int, std::vector<int>> constrained_vertices;
    std::unordered_set<int> constrained_vertices_set;
    for (auto link_el = constraints_el->FirstChildElement("link");
         link_el != nullptr;
         link_el = link_el->NextSiblingElement("link")) {

        std::string link_name = link_el->Attribute("name");
        int link_idx = std::find(art.names.begin(), art.names.end(), link_name) - art.names.begin();
        constrained_vertices.insert({link_idx, {}});
        auto vertices_el = link_el->FirstChildElement("vertices");
        std::stringstream ss(vertices_el->GetText());
        std::string token;
        while (ss >> token) {
            int vidx = std::stoi(token);
            constrained_vertices[link_idx].push_back(vidx);
            constrained_vertices_set.insert(vidx);
        }
    }

    num_constrained_vertices = 0;
    std::vector<int> index_map(soft_body_obj.vertices.size(), -1);
    for (auto& [link_idx, indices] : constrained_vertices) {
        num_constrained_vertices += indices.size();
    }
    constrained_idx_start = soft_body_obj.vertices.size() - num_constrained_vertices;
    int current_index = 0;
    for (int i = 0; i < soft_body_obj.vertices.size(); i++) {
        if (constrained_vertices_set.count(i) == 0) {
            index_map[i] = current_index;
            current_index++;
        }
    }
    // TODO: Debug on why this assert isn't true
    assert(current_index == constrained_idx_start);
    for (auto& [link_idx, indices] : constrained_vertices) {
        std::pair<int, int> vertices_range;
        vertices_range.first = current_index;
        for (int i : indices) {
            index_map[i] = current_index;
            current_index++;
        }
        vertices_range.second = current_index;
        constrained_vertices_range[link_idx] = vertices_range;
    }

    sb.vertices.resize(soft_body_obj.vertices.size());
    for (int old_idx = 0; old_idx < index_map.size(); old_idx++) {
        int new_idx = index_map[old_idx];
        sb.vertices[new_idx] = soft_body_obj.vertices[old_idx];
    }
    sb.tetrahedrons.resize(soft_body_obj.tetrahedrons.size());
    for (int i = 0; i < soft_body_obj.tetrahedrons.size(); i++) {
        auto old_tet = soft_body_obj.tetrahedrons[i];
        sb.tetrahedrons[i][0] = index_map[old_tet[0]];
        sb.tetrahedrons[i][1] = index_map[old_tet[1]];
        sb.tetrahedrons[i][2] = index_map[old_tet[2]];
        sb.tetrahedrons[i][3] = index_map[old_tet[3]];
    }

    gen_surface_triangles_from_tet_mesh(sb.tetrahedrons, sb.triangles);
}

void soft_body_precomputation(SoftBodyWithArtData& body, const ADMMConstraints& constraints, real dt) {
    soft_body_precomputation(body.sb, constraints, dt);
}

void admm_dynamics_with_art(const SoftBodyWithArtData& data, const ADMMConstraints& constraints,
                            real dt, const real* sb_f, const real* art_f,
                            INOUT real* sb_pos, INOUT real* sb_vel, INOUT real* sb_f_contact,
                            INOUT real* art_pos, INOUT real* art_vel, INOUT real* art_f_contact) {

    using namespace Eigen;
    using real = artsim::real;
    using MatrixXr = Matrix<real, Dynamic, Dynamic>;
    using VectorXr = Matrix<real, Dynamic, 1>;

    auto& sb = data.sb;
    auto& art = data.art;
    int num_vertices = sb.vertices.size();
    int num_art_links = art.get_num_joints();
    int num_art_pos_dofs = art.get_num_pos_dofs();
    int num_art_vel_dofs = art.get_num_vel_dofs();
    int num_free_vertices = data.constrained_idx_start;
    int num_constrained_vertices = sb.vertices.size() - data.constrained_idx_start;
    int N_s = num_vertices, N_f = num_free_vertices, N_c = num_constrained_vertices, N_r = num_art_vel_dofs;

    Map<VectorXr> x_s(sb_pos, 3*num_vertices);
    Map<VectorXr> v_s(sb_vel, 3*num_vertices);
    Map<VectorXr> x_r(art_pos, 3*num_art_pos_dofs);
    Map<VectorXr> v_r(art_vel, 3*num_art_vel_dofs);
    Map<const VectorXr> f_s_orig(sb_f, 3*N_s);
    Map<const VectorXr> f_r_orig(art_f, 3*N_r);
    VectorXr x_s_orig = x_s;
    VectorXr x_bar = x_s + dt * v_s + dt*dt * sb.M_LDLt.solve(f_s_orig);

    VectorXr f_c = VectorXr::Zero(N_c);
    VectorXr f_r = VectorXr::Zero(N_r);
    VectorXr x_tilde;

    std::vector<glm::tmat3x3<real>> u(sb.tetrahedrons.size(), glm::tmat3x3<real>(0.0));
    std::vector<glm::tmat3x3<real>> z(sb.tetrahedrons.size());
    std::vector<glm::tmat3x3<real>> p(sb.tetrahedrons.size());
    glm::tvec3<real>* V = (glm::tvec3<real>*) x_s.data();

    for (int iter = 0; iter < 5; iter++) {
        x_tilde.noalias() = x_bar;
        x_tilde.rightCols(N_c).noalias() += dt * dt * f_c;

        // Local solve
#define X(CTYPE, CFIELD) \
        admm_volume_constraint_local_solve(sb, constraints.CFIELD.data(), constraints.CFIELD.size(), V, OUT z.data(), OUT u.data(), OUT p.data());
        ADMM_VOLUME_CONSTRAINTS
#undef X

        // Global solve
        VectorXr b = sb.M * x_tilde;

#define X(CTYPE, CFIELD) \
        global_solve_modify_b(sb, constraints.CFIELD.data(), constraints.CFIELD.size(), dt, p.data(), OUT b.data());
        ADMM_VOLUME_CONSTRAINTS
#undef X

        // Calculate vertex jacobians
        std::vector<ttransform<real>> link_trans(num_art_links), joint_trans(num_art_links);
        calc_transforms(art, art_pos, link_trans.data(), joint_trans.data());

        std::vector<tscrew<real>> global_joint_S(num_art_links);
        calc_S(art, art_pos, global_joint_S.data());
        for (int link_idx = 0; link_idx < num_art_links; link_idx++) {
            global_joint_S[link_idx] = Ad(joint_trans[link_idx], global_joint_S[link_idx]);
        }

        Matrix<real, Dynamic, Dynamic> J_cr(3*N_c, N_r);
        J_cr.setZero();

        for (auto& [link_idx, vidx_range] : data.constrained_vertices_range) {
            for (int j = art.joint_vel_dof_starts[link_idx]; j < art.joint_vel_dof_starts[link_idx+1]; j++) {
                auto joint_axis = global_joint_S[j].w;
                for (int vidx = vidx_range.first; vidx < vidx_range.second; vidx++) {
                    auto T_v = sb.vertices[vidx] - joint_trans[link_idx].v;
                    auto vel = glm::cross(joint_axis, T_v);
                    J_cr(3*vidx+0, j) = vel[0];
                    J_cr(3*vidx+1, j) = vel[1];
                    J_cr(3*vidx+2, j) = vel[2];
                }
            }
        }

        // Calculate inverse of articulation matrix M_r^{-1}
        Eigen::Matrix<real, Dynamic, Dynamic> M_r_inv;
        dynmat_view<real> M_r_inv_view(M_r_inv.data(), num_art_vel_dofs, num_art_vel_dofs);
        dynmat<real> identity(num_art_vel_dofs, IDENTITY);
        multiply_inverse_mass_matrix(art, dt, art_pos, identity.to_view(), OUT M_r_inv_view);

        // TODO: Do the rest of the f***ing owl!
        //       Construct the linear system
        //       Solve it via Uzawa Conjugate Gradient
    }

}

}