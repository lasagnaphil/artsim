//
// Created by lasagnaphil on 2/16/21.
//

#include "artsim/soft_body_with_art.h"
#include "artsim/soft_body.h"
#include "artsim/dynamics.h"
#include "artsim/utils/xml.h"
#include "artsim/math/fastsvd.h"

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
    fs::path soft_body_file = folder / soft_body_mesh_el->GetText();
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
    auto art_file = folder / articulation_el->GetText();
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
        std::cout << num_constrained_vertices << std::endl;
    }
    std::cout << std::endl;
    constrained_idx_start = soft_body_obj.vertices.size() - num_constrained_vertices;
    int current_index = 0;
    for (int i = 0; i < soft_body_obj.vertices.size(); i++) {
        if (constrained_vertices_set.count(i) == 0) {
            index_map[i] = current_index;
            current_index++;
        }
    }
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

    // Calculate physics parameters
    int num_art_links = art.links.size();
    std::vector<real> q_zero(art.get_num_pos_dofs());
    set_zero_pose(art, OUT q_zero.data());

    std::vector<ttransform<real>> T_link_global(num_art_links), T_joint_global(num_art_links);
    calc_transforms(art, q_zero.data(), OUT T_link_global.data(), OUT T_joint_global.data());

    for (auto& [link_idx, vidx_range] : constrained_vertices_range) {
        int joint_vel_dof_start = art.joint_vel_dof_starts[link_idx];
        int joint_vel_dofs = art.joint_vel_dofs[link_idx];
        for (int j = joint_vel_dof_start; j < joint_vel_dof_start + joint_vel_dofs; j++) {
            for (int vidx = vidx_range.first; vidx < vidx_range.second; vidx++) {
                auto vert_trans = ttransform<real>(sb.vertices[vidx]);
                constrained_vertices_offset[vidx] = vert_trans / T_joint_global[link_idx];
            }
        }
    }
}

void soft_body_precomputation(SoftBodyWithArtData& body, const ADMMConstraints& constraints, real dt) {
    precomputation_essentials(body.sb);
    update_system_matrix(body.sb, constraints, dt, OUT body.sb.A);
    body.sb.A_LDLt.analyzePattern(body.sb.A);
    body.sb.A_LDLt.factorize(body.sb.A);
}

void admm_dynamics_with_art(const SoftBodyWithArtData& data, const ADMMConstraints& constraints,
                            real dt, glm::rvec3 gravity, const real* sb_f, const real* art_f,
                            INOUT real* sb_pos, INOUT real* sb_vel,
                            INOUT real* art_pos, INOUT real* art_vel) {

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

    // TODO: investigate if jacobian calculation is bugged!!!
    // Calculate vertex jacobians
    std::vector<ttransform<real>> link_trans(num_art_links), joint_trans(num_art_links);
    calc_transforms(art, art_pos, link_trans.data(), joint_trans.data());

    std::vector<tscrew<real>> joint_S(N_r);
    calc_S(art, art_pos, joint_S.data());

    Matrix<real, Dynamic, Dynamic> J_cr(3*N_c, N_r);
    J_cr.setZero();

    for (auto& [link_idx, vidx_range] : data.constrained_vertices_range) {
        int idx = link_idx;
        while (idx != -1) {
            int joint_vel_dof_start = art.joint_vel_dof_starts[idx];
            int joint_vel_dofs = art.joint_vel_dofs[idx];
            for (int j = joint_vel_dof_start; j < joint_vel_dof_start + joint_vel_dofs; j++) {
                for (int vidx = vidx_range.first; vidx < vidx_range.second; vidx++) {
                    glmx::ttransform<real> T_v = joint_trans[link_idx] * data.constrained_vertices_offset.at(vidx);
                    glmx::tscrew<real> S_prime = Ad(joint_trans[idx] / T_v, joint_S[j]);
                    rvec3 S_v = T_v.R * S_prime.v;
                    J_cr(3*(vidx-N_f)+0, j) = S_v[0];
                    J_cr(3*(vidx-N_f)+1, j) = S_v[1];
                    J_cr(3*(vidx-N_f)+2, j) = S_v[2];
                }
            }
            idx = art.parents[idx];
        }
    }

    // Calculate articulation matrix M_r
    MatrixXr M_r(N_r, N_r);
    dynmat_view<real> M_r_view(M_r.data(), N_r, N_r);
    mass_matrix(art, dt, art_pos, M_r_view);

    // Calculate inverse of articulation matrix M_r^{-1}
    MatrixXr M_r_inv(N_r, N_r);
    dynmat_view<real> M_r_inv_view(M_r_inv.data(), N_r, N_r);
    dynmat<real> identity(num_art_vel_dofs, IDENTITY);
    multiply_inverse_mass_matrix(art, dt, art_pos, identity.to_view(), OUT M_r_inv_view);

    // Calculate other matrices related to articulation
    MatrixXr M_r_inv_J_cr_T = M_r_inv * J_cr.transpose();

    Map<VectorXr> x_s(sb_pos, 3*N_s);
    Map<VectorXr> x_f(sb_pos, 3*N_f);
    Map<VectorXr> x_c(sb_pos + 3*N_f, 3*N_c);
    Map<VectorXr> x_r(art_pos, num_art_pos_dofs);

    Map<VectorXr> v_s(sb_vel, 3*N_s);
    Map<VectorXr> v_c(sb_vel + 3*N_f, 3*N_c);
    Map<VectorXr> v_r(art_vel, N_r);
    Map<const VectorXr> f_s_ext(sb_f, 3*N_s);
    Map<const VectorXr> f_r_ext(art_f, N_r);

    // Add gravity to total force
    VectorXr f_s_tot = f_s_ext;
    auto f_s_tot_ptr = (glm::rvec3*) f_s_tot.data();
    for (int t = 0; t < data.sb.tetrahedrons.size(); t++) {
        glm::ivec4 tet = data.sb.tetrahedrons[t];
        glm::rvec3 f_g = (1. / 4.) * data.sb.props.density * data.sb.W[t] * gravity;
        f_s_tot_ptr[tet[0]] += f_g;
        f_s_tot_ptr[tet[1]] += f_g;
        f_s_tot_ptr[tet[2]] += f_g;
        f_s_tot_ptr[tet[3]] += f_g;
    }

    VectorXr x_s_orig = x_s;
    VectorXr v_s_tilde = v_s + dt * sb.M_LDLt.solve(f_s_tot);

    VectorXr x_r_orig = x_r;
    VectorXr v_r_dot(N_r);
    featherstone_forward_dynamics(art, gravity, dt, nullptr, art_pos, art_vel, art_f, OUT v_r_dot.data());
    VectorXr v_r_tilde = v_r + dt * v_r_dot;

    // v_s_tilde.bottomRows(3*N_c) = J_cr * v_r_tilde;

    v_s = v_s_tilde;
    v_r = v_r_tilde;
    x_s = x_s_orig + dt*v_s;

    std::vector<glm::tmat3x3<real>> u(sb.tetrahedrons.size(), glm::tmat3x3<real>(0.0));
    std::vector<glm::tmat3x3<real>> z(sb.tetrahedrons.size(), glm::tmat3x3<real>(0.0));
    std::vector<glm::tmat3x3<real>> z_prev(sb.tetrahedrons.size());
    std::vector<glm::tmat3x3<real>> p(sb.tetrahedrons.size());
    std::vector<glm::tmat3x3<real>> F(sb.tetrahedrons.size());
    std::vector<glmx::SVD_mats<real>> F_svd(sb.tetrahedrons.size());

    std::cout << std::endl << "Starting ADMM loop" << std::endl;
    for (int iter = 0; iter < 10; iter++) {
        z_prev = z;

        // Local solve
#define X(CTYPE, CFIELD) \
        admm_volume_constraint_local_solve(sb, constraints.CFIELD.data(), constraints.CFIELD.size(), \
            (glm::rvec3*)x_s.data(), OUT z.data(), OUT u.data(), OUT F.data(), OUT F_svd.data());
        ADMM_VOLUME_CONSTRAINTS
#undef X

        // Global solve
        VectorXr b_s = sb.M * v_s_tilde;
        VectorXr b_r = M_r * v_r_tilde;

#define X(CTYPE, CFIELD) \
        admm_volume_constraint_update_b(sb, constraints.CFIELD.data(), constraints.CFIELD.size(), \
            dt, z.data(), u.data(), (glm::rvec3*)x_s_orig.data(), OUT b_s.data());
        ADMM_VOLUME_CONSTRAINTS
#undef X

        VectorXr f_c = VectorXr::Zero(3*N_c);

        b_s.bottomRows(3*N_c) -= f_c;
        b_r += J_cr.transpose() * f_c;

        v_s = sb.A_LDLt.solve(b_s);
        v_r = M_r_inv * b_r;

        VectorXr r_f = v_c - J_cr * v_r;
        VectorXr s_f = r_f;
        VectorXr s_v(3*N_s + N_r);

        int uzawa_iter = 0;
        while (r_f.squaredNorm() > 1e-4) {
            VectorXr s_f_s(3*N_s);
            s_f_s.topRows(3*N_f).setZero();
            s_f_s.bottomRows(3*N_c) = s_f;
            s_v.topRows(3*N_s) = sb.A_LDLt.solve(s_f_s);
            s_v.bottomRows(N_r) = -M_r_inv_J_cr_T * s_f;
            VectorXr a_f = s_v.middleRows(3*N_f, 3*N_c) - J_cr * s_v.bottomRows(N_r);
            real s_f_a_f = s_f.dot(a_f);
            real alpha = s_f.dot(r_f) / s_f_a_f;
            v_s -= alpha * s_v.topRows(3*N_s);
            v_r -= alpha * s_v.bottomRows(N_r);
            f_c += alpha * s_f;
            r_f -= alpha * a_f;
            real beta = r_f.dot(a_f) / s_f_a_f;
            s_f = r_f - beta*s_f;
            uzawa_iter++;
        }

        std::cout << "Uzawa iter converged in " << uzawa_iter << " iters! " <<
            "(residual = " << r_f.norm() << ")" << std::endl;

        x_s = x_s_orig + dt*v_s;

        real primal_res_sq = 0, dual_res_sq = 0;
#define X(CTYPE, CFIELD) \
        admm_volume_constraint_update_residuals(sb, constraints.CFIELD.data(), constraints.CFIELD.size(), \
            z_prev.data(), z.data(), (glm::rvec3*)x_s.data(), INOUT primal_res_sq, INOUT dual_res_sq);
        ADMM_VOLUME_CONSTRAINTS
#undef X

        std::cout << "primal_res = " << sqrt(primal_res_sq) << ", dual_res = " << sqrt(dual_res_sq) << std::endl;
    }

    // Baumgarte stabilization
    /*
    VectorXr dV(3*N_s);
    dV.topRows(3*N_f).setZero();
    dV.bottomRows(3*N_c) = v_c - J_cr * v_r;
    real k_baum = 1;
    VectorXr f_baum = -k_baum * sb.M_LDLt.solve(dV);
    v_s += dt*f_baum;
    x_s += dt*dt*f_baum;
     */

    // integrate_implicit_euler(art, dt, nullptr, x_r.data(), v_r.data());

    // Project constrained velocities to articulation
    // v_c = J_cr * v_r;
    // x_s = x_s_orig + dt*v_s;

    integrate_implicit_euler(art, dt, nullptr, x_r.data(), v_r.data());

    // Project constrained positions to articulation
    calc_transforms(art, art_pos, link_trans.data(), joint_trans.data());
    for (auto& [link_idx, vidx_range] : data.constrained_vertices_range) {
        for (int vidx = vidx_range.first; vidx < vidx_range.second; vidx++) {
            auto T_v = data.constrained_vertices_offset.at(vidx);
            auto T = joint_trans[link_idx] * T_v;
            sb_pos[3*(vidx) + 0] = T.v[0];
            sb_pos[3*(vidx) + 1] = T.v[1];
            sb_pos[3*(vidx) + 2] = T.v[2];
        }
    }
}

}