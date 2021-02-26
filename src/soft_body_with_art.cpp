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
    int N_f = body.constrained_idx_start;
    int N_s = body.sb.vertices.size();
    for (int i = 3*N_f; i < 3*N_s; i++) {
        body.sb.A.coeffRef(i, i) += body.k_c;
    }

    body.sb.A_LDLt.analyzePattern(body.sb.A);
    body.sb.A_LDLt.factorize(body.sb.A);
}

template <class Constraint>
void admm_dynamics_with_art_volume_local_solve(
        const SoftBodyData& body, const Constraint* constraints, uint32_t num_constraints,
        const glm::tvec3<real>* x,
        OUT glm::tmat3x3<real>* z, OUT glm::tmat3x3<real>* u,
        OUT glm::tmat3x3<real>* F, OUT glmx::SVD_mats<real>* F_svd) {

// #pragma omp parallel for schedule(static)
    for (int cidx = 0; cidx < num_constraints; cidx++) {
        auto& c = constraints[cidx];
        glm::ivec4 tet = body.tetrahedrons[c.tet_id];
        glm::rmat3 x_mat(x[tet[0]] - x[tet[3]], x[tet[1]] - x[tet[3]], x[tet[2]] - x[tet[3]]);
        F[c.tet_id] = x_mat * body.B_m[c.tet_id] + u[c.tet_id];
    }

    glmx::fastsvd(F, num_constraints, F_svd);

// #pragma omp parallel for schedule(static)
    for (int cidx = 0; cidx < num_constraints; cidx++) {
        auto& c = constraints[cidx];
        F_svd[c.tet_id].Sigma = proximal_eigvec(F_svd[c.tet_id].Sigma, c);
        z[c.tet_id] = F_svd[c.tet_id].recover_matrix();
        u[c.tet_id] = F[c.tet_id] - z[c.tet_id];
    }
}

template <class Constraint>
void admm_dynamics_with_art_update_b(
        const SoftBodyData& body, const Constraint* constraints, uint32_t num_constraints,
        real dt, const glm::tmat3x3<real>* z, const glm::tmat3x3<real>* u_s, const glm::tvec3<real>* x0,
        INOUT real* b) {
    for (int cidx = 0; cidx < num_constraints; cidx++) {
        auto& c = constraints[cidx];
        glm::ivec4 tet = body.tetrahedrons[c.tet_id];
        glm::rmat3 p = z[c.tet_id] - u_s[c.tet_id];
        auto& D_i = body.D[c.tet_id];
        real k_s = dt * dt * c.k * body.W[c.tet_id];
        for (int j = 0; j < 4; j++) {
            glm::tvec3<real> db = k_s * dt * (p * D_i[j]);
            b[3*tet[j]+0] += db[0];
            b[3*tet[j]+1] += db[1];
            b[3*tet[j]+2] += db[2];
        }
        for (int j = 0; j < 4; j++) {
            for (int k = 0; k < 4; k++) {
                glm::tvec3<real> db = k_s * dt * glm::dot(body.D[c.tet_id][j], body.D[c.tet_id][k]) * x0[tet[k]];
                b[3*tet[j]+0] -= db[0];
                b[3*tet[j]+1] -= db[0];
                b[3*tet[j]+2] -= db[0];
            }
        }
    }
}

// TODO: Need to fix local updates on soft body not working
void admm_dynamics_with_art(const SoftBodyWithArtData& data, const ADMMConstraints& constraints,
                            real dt, const real* sb_f, const real* art_f,
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
                auto S_j = joint_S[j];
                for (int vidx = vidx_range.first; vidx < vidx_range.second; vidx++) {
                    glmx::ttransform<real> T_v = joint_trans[link_idx] * data.constrained_vertices_offset.at(vidx);
                    glmx::tscrew<real> S_prime = Ad(joint_trans[idx] / T_v, S_j);
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

    Map<VectorXr> x_s(sb_pos, 3*N_s);
    Map<VectorXr> x_f(sb_pos, 3*N_f);
    Map<VectorXr> x_c(sb_pos + 3*N_f, 3*N_c);
    Map<VectorXr> x_r(art_pos, num_art_pos_dofs);

    Map<VectorXr> v_s(sb_vel, 3*N_s);
    Map<VectorXr> v_c(sb_vel + 3*N_f, 3*N_c);
    Map<VectorXr> v_r(art_vel, N_r);
    Map<const VectorXr> f_s_ext(sb_f, 3*N_s);
    Map<const VectorXr> f_r_ext(art_f, N_r);

    VectorXr x_s_orig = x_s;
    VectorXr v_s_tilde = v_s + dt * sb.M_LDLt.solve(f_s_ext);

    VectorXr v_r_dot(N_r);
    featherstone_forward_dynamics(art, glm::rvec3(0), dt, nullptr, art_pos, art_vel, art_f, OUT v_r_dot.data());
    VectorXr v_r_tilde = v_r + dt * v_r_dot;

    std::vector<glm::tmat3x3<real>> u_s(sb.tetrahedrons.size(), glm::tmat3x3<real>(0.0));
    VectorXr u_c = VectorXr::Zero(3*N_c);
    std::vector<glm::tmat3x3<real>> z(sb.tetrahedrons.size());
    std::vector<glm::tmat3x3<real>> p(sb.tetrahedrons.size());
    std::vector<glm::tmat3x3<real>> F(sb.tetrahedrons.size());
    std::vector<glmx::SVD_mats<real>> F_svd(sb.tetrahedrons.size());

    std::cout << "Starting ADMM loop" << std::endl;
    v_s = v_s_tilde;
    v_r = v_r_tilde;
    for (int iter = 0; iter < 10; iter++) {
        VectorXr x_s_pred = x_s_orig + dt*v_s;

        // Local solve
#define X(CTYPE, CFIELD) \
        admm_dynamics_with_art_volume_local_solve(sb, constraints.CFIELD.data(), constraints.CFIELD.size(), \
            (glm::rvec3*)x_s_pred.data(), OUT z.data(), OUT u_s.data(), OUT F.data(), OUT F_svd.data());
        ADMM_VOLUME_CONSTRAINTS
#undef X
        u_c += (v_c - J_cr * v_r);

        // Global solve
        VectorXr b(3*N_s + N_r);
        b.topRows(3*N_s) = sb.M * v_s_tilde;

#define X(CTYPE, CFIELD) \
        admm_dynamics_with_art_update_b(sb, constraints.CFIELD.data(), constraints.CFIELD.size(), \
            dt, z.data(), u_s.data(), (glm::rvec3*)x_s_orig.data(), OUT b.data());
        ADMM_VOLUME_CONSTRAINTS
#undef X

        real k_c = data.k_c;

        b.middleRows(3*N_f, 3*N_c) -= k_c * u_c;
        b.bottomRows(N_r) = M_r * v_r_tilde + k_c * J_cr.transpose() * u_c;
        // std::cout << b.middleRows(3*N_f, 3*N_c).transpose() << std::endl;
        // std::cout << b.bottomRows(N_r).transpose() << std::endl;

        // MatrixXr Linv_J_cr(3*N_s, N_r);
        // Linv_J_cr.topRows(3*N_f).setZero();
        // Linv_J_cr.bottomRows(3*N_c) = J_cr;
        // sb.A_LDLt.matrixL().solveInPlace(Linv_J_cr);
        // MatrixXr A_r = M_r + k_c * J_cr.transpose() * J_cr;
        // MatrixXr A_r_prime = A_r - (k_c*k_c) * Linv_J_cr.transpose() * sb.A_LDLt.vectorD().asDiagonal() * Linv_J_cr;
        MatrixXr A_r = M_r + k_c * J_cr.transpose() * J_cr;
        MatrixXr A_r_prime = A_r - (k_c*k_c) * J_cr.transpose() * sb.A_LDLt.solve(J_cr);

        VectorXr A_s_inv_b_s = sb.A_LDLt.solve(b.topRows(3*N_s));
        VectorXr b_r_prime = b.bottomRows(N_r) - data.k_c * J_cr.transpose() * A_s_inv_b_s.bottomRows(3*N_c);

        // std::cout << "b_s: " << b.topRows(3*N_s).transpose() << std::endl;
        // std::cout << "b_r_prime: " << b_r_prime.transpose() << std::endl;

        v_r = A_r_prime.ldlt().solve(b_r_prime);

        VectorXr b_s_prime = b.topRows(3*N_s);
        b_s_prime.bottomRows(3*N_c) -= k_c * J_cr * v_r;
        v_s = sb.A_LDLt.solve(b_s_prime);

        std::cout << "coupling error: " << (v_c - J_cr * v_r).norm() << std::endl;
    }

    x_s = x_s_orig + dt*v_s;
    integrate_implicit_euler(art, dt, nullptr, x_r.data(), v_r.data());

    // Project constrained vertices to articulation
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