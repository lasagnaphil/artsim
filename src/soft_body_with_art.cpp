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
    using namespace Eigen;
    using real = artsim::real;
    using MatrixXr = Matrix<real, Dynamic, Dynamic>;
    using VectorXr = Matrix<real, Dynamic, 1>;

    soft_body_precomputation(body.sb, constraints, dt);

    auto& A = body.sb.A;
    int N_f = body.constrained_idx_start;
    int N_c = body.sb.vertices.size() - body.constrained_idx_start;
    MatrixXr A_cc(3*N_c, 3*N_c);
    A_cc.setZero();

    for (int k=0; k<A.outerSize(); ++k) {
        for (SparseMatrix<real>::InnerIterator it(A,k); it; ++it) {
            if (it.row() >= 3*N_f && it.col() >= 3*N_f) {
                A_cc(it.row()-3*N_f, it.col()-3*N_f) = it.value();
            }
        }
    }

    VectorXr eigvals = A_cc.eigenvalues().real();
    body.A_sigma_min = A_cc.minCoeff();
    body.A_sigma_max = A_cc.maxCoeff();
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

    Map<VectorXr> x_s(sb_pos, 3*N_s);
    Map<VectorXr> v_s(sb_vel, 3*N_s);
    Map<VectorXr> x_f(sb_pos, 3*N_f);
    Map<VectorXr> x_c(sb_pos + 3*N_f, 3*N_c);
    Map<VectorXr> x_r(art_pos, num_art_pos_dofs);
    Map<VectorXr> v_r(art_vel, N_r);
    Map<const VectorXr> f_s_ext(sb_f, 3*N_s);
    Map<const VectorXr> f_r_ext(art_f, N_r);

    VectorXr x_s_orig = x_s;
    VectorXr x_c_orig = x_c;
    VectorXr x_s_bar = x_s + dt * v_s + dt*dt * sb.M_LDLt.solve(f_s_ext);

    VectorXr v_r_dot(N_r);
    featherstone_forward_dynamics(art, glm::rvec3(0), dt, nullptr, art_pos, art_vel, art_f, OUT v_r_dot.data());
    v_r += dt * v_r_dot;
    // integrate_implicit_euler(art, dt, v_r_dot.data(), art_pos, art_vel);

    VectorXr f_c = Map<VectorXr>(sb_f_contact, 3*N_c);
    VectorXr f_r = Map<VectorXr>(art_f_contact, N_r);
    f_c.setZero(); f_r.setZero();

    // Calculate vertex jacobians
    std::vector<ttransform<real>> link_trans(num_art_links), joint_trans(num_art_links);
    calc_transforms(art, art_pos, link_trans.data(), joint_trans.data());

    std::vector<tscrew<real>> joint_S(N_r);
    calc_S(art, art_pos, joint_S.data());

    Matrix<real, Dynamic, Dynamic> J_cr(3*N_c, N_r);
    J_cr.setZero();

    for (auto& [link_idx, vidx_range] : data.constrained_vertices_range) {
        /*
        std::vector<tscrew<real>> J(num_art_links);
        for (int vidx = vidx_range.first; vidx < vidx_range.second; vidx++) {
            glmx::ttransform<real> T_v = data.constrained_vertices_offset.at(vidx);
            calc_space_jacobian(art, link_idx, T_v, joint_S.data(), joint_trans.data(), J.data());
        }
         */
        int idx = link_idx;
        while (idx != -1) {
            int joint_vel_dof_start = art.joint_vel_dof_starts[idx];
            int joint_vel_dofs = art.joint_vel_dofs[idx];
            for (int j = joint_vel_dof_start; j < joint_vel_dof_start + joint_vel_dofs; j++) {
                auto S_j = joint_S[j];
                for (int vidx = vidx_range.first; vidx < vidx_range.second; vidx++) {
                    glmx::ttransform<real> T_v = data.constrained_vertices_offset.at(vidx);
                    glmx::tscrew<real> S_prime = Ad(joint_trans[idx] * T_v, S_j);
                    J_cr(3*(vidx-N_f)+0, j) = S_prime.v[0];
                    J_cr(3*(vidx-N_f)+1, j) = S_prime.v[1];
                    J_cr(3*(vidx-N_f)+2, j) = S_prime.v[2];
                }
            }
            idx = art.parents[idx];
        }
    }

    VectorXr x_c_bar = x_c_orig + dt*J_cr*v_r;

    std::vector<glm::tmat3x3<real>> u(sb.tetrahedrons.size(), glm::tmat3x3<real>(0.0));
    std::vector<glm::tmat3x3<real>> z(sb.tetrahedrons.size());
    std::vector<glm::tmat3x3<real>> p(sb.tetrahedrons.size());
    std::vector<glm::tmat3x3<real>> F(sb.tetrahedrons.size());
    std::vector<glmx::SVD_mats<real>> F_svd(sb.tetrahedrons.size());
    glm::tvec3<real>* V = (glm::tvec3<real>*) x_s.data();

    std::cout << "Starting ADMM loop" << std::endl;
    for (int iter = 0; iter < 10; iter++) {
        VectorXr x_s_tilde = x_s_bar;
        x_s_tilde.bottomRows(3*N_c) += dt * dt * f_c;

        // Local solve
#define X(CTYPE, CFIELD) \
        admm_volume_constraint_local_solve_fast(sb, constraints.CFIELD.data(), constraints.CFIELD.size(), V, \
            OUT z.data(), OUT u.data(), OUT p.data(), OUT F.data(), OUT F_svd.data());
        ADMM_VOLUME_CONSTRAINTS
#undef X

        // Global solve
        VectorXr b = sb.M * x_s_tilde;

#define X(CTYPE, CFIELD) \
        global_solve_modify_b(sb, constraints.CFIELD.data(), constraints.CFIELD.size(), dt, p.data(), OUT b.data());
        ADMM_VOLUME_CONSTRAINTS
#undef X

        // Calculate inverse of articulation matrix M_r^{-1}
        MatrixXr M_r_inv(N_r, N_r);
        dynmat_view<real> M_r_inv_view(M_r_inv.data(), N_r, N_r);
        dynmat<real> identity(num_art_vel_dofs, IDENTITY);
        multiply_inverse_mass_matrix(art, dt, art_pos, identity.to_view(), OUT M_r_inv_view);

        // Calculate Delassus matrix.
        MatrixXr M_d = J_cr * M_r_inv * J_cr.transpose();
        // LDLT<MatrixXr> M_d_T_M_d_LDLt = (M_d.transpose() * M_d).ldlt();
        // TODO: To get optimal alpha, (approximately) calculate min/max eigenvalues of schur complement matrix.
        // real alpha = real(2) / (data.A_sigma_min + data.A_sigma_max);
        // std::cout << "alpha = " << alpha << std::endl;
        real alpha = 0.005;

        std::cout << "Starting Uzawa CG" << std::endl;
        for (int uzawa_iter = 0; uzawa_iter < 10; uzawa_iter++) {
            /*
            f_c = M_d_T_M_d_LDLt.solve(M_d.transpose() * (x_c_bar - x_c) / (dt*dt));
            VectorXr b_bar = b;
            b_bar.bottomRows(3*N_c) += (dt*dt)*f_c;
            x_s = sb.A_LDLt.solve(b_bar);
             */
            VectorXr b_bar = b;
            b_bar.bottomRows(3*N_c) += (dt*dt)*f_c;
            x_s = sb.A_LDLt.solve(b_bar);

            f_c -= alpha * ((x_c - x_c_bar)/(dt*dt) + M_d * f_c);

            // VectorXr error = (M_d * f_c + (x_c - x_c_bar) / (dt*dt));
            // std::cout << "Error: " << error.lpNorm<2>() << std::endl;
            // std::cout << "Iteration " << uzawa_iter << ": " << std::endl;
            // std::cout << "b_bar: " << b_bar.transpose() << std::endl;
            // std::cout << "x_s: " << x_s.transpose() << std::endl;
            // std::cout << "f_c: " << f_c.transpose() << std::endl;
            // std::cout << "f_r: " << f_r.transpose() << std::endl;
        }
        f_r = -J_cr.transpose() * f_c;

        VectorXr error = (M_d * f_c + (x_c - x_c_bar) / (dt*dt));
        std::cout << "Error: " << error.lpNorm<2>() << std::endl;
    }

    v_s = (x_s - x_s_orig) / dt;

    VectorXr f_r_total = f_r_ext + f_r;
    std::cout << f_r_total.transpose() << std::endl;
    featherstone_forward_dynamics(art, glm::rvec3(0), dt, nullptr, art_pos, art_vel, f_r_total.data(),
                                  OUT v_r_dot.data());
    integrate_implicit_euler(art, dt, v_r_dot.data(), art_pos, art_vel);

    x_c = x_c_orig + dt*J_cr*v_r;

    for (auto& [link_idx, vidx_range] : data.constrained_vertices_range) {
        /*
        std::vector<tscrew<real>> J(num_art_links);
        for (int vidx = vidx_range.first; vidx < vidx_range.second; vidx++) {
            glmx::ttransform<real> T_v = data.constrained_vertices_offset.at(vidx);
            calc_space_jacobian(art, link_idx, T_v, joint_S.data(), joint_trans.data(), J.data());
        }
         */
        for (int vidx = vidx_range.first; vidx < vidx_range.second; vidx++) {
            auto T_v = data.constrained_vertices_offset.at(vidx);
            auto T = joint_trans[link_idx] * T_v;
            x_c(3*(vidx-N_f) + 0) = T.v[0];
            x_c(3*(vidx-N_f) + 1) = T.v[1];
            x_c(3*(vidx-N_f) + 2) = T.v[2];
        }
    }
}

}