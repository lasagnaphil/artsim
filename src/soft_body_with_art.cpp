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

using namespace Eigen;
using real = artsim::real;
using MatrixXr = Matrix<artsim::real, Dynamic, Dynamic>;
using VectorXr = Matrix<artsim::real, Dynamic, 1>;

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

    int num_constrained_vertices = 0;
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

    vertices.resize(soft_body_obj.vertices.size());
    for (int old_idx = 0; old_idx < index_map.size(); old_idx++) {
        int new_idx = index_map[old_idx];
        vertices[new_idx] = soft_body_obj.vertices[old_idx];
    }
    tetrahedrons.resize(soft_body_obj.tetrahedrons.size());
    for (int i = 0; i < soft_body_obj.tetrahedrons.size(); i++) {
        auto old_tet = soft_body_obj.tetrahedrons[i];
        tetrahedrons[i][0] = index_map[old_tet[0]];
        tetrahedrons[i][1] = index_map[old_tet[1]];
        tetrahedrons[i][2] = index_map[old_tet[2]];
        tetrahedrons[i][3] = index_map[old_tet[3]];
    }

    gen_surface_triangles_from_tet_mesh(tetrahedrons, triangles);

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
                auto vert_trans = ttransform<real>(vertices[vidx]);
                constrained_vertices_offset[vidx] = vert_trans / T_joint_global[link_idx];
            }
        }
    }

    int num_vertices = vertices.size();
    int num_art_pos_dofs = art.get_num_pos_dofs();
    int num_art_vel_dofs = art.get_num_vel_dofs();
    int num_free_vertices = constrained_idx_start;
    N_s = num_vertices, N_f = num_free_vertices, N_c = num_constrained_vertices, N_r = num_art_vel_dofs;

}

void soft_body_precomputation(SoftBodyWithArtData& body, const ADMMConstraints& constraints, real dt) {
    body.B_m.resize(body.tetrahedrons.size());
    body.W.resize(body.tetrahedrons.size());
    body.D.resize(body.tetrahedrons.size());
    for (int i = 0; i < body.tetrahedrons.size(); i++) {
        auto& V = body.vertices;
        glm::ivec4& tet = body.tetrahedrons[i];
        glm::tmat3x3<real> D_m(V[tet[0]] - V[tet[3]], V[tet[1]] - V[tet[3]], V[tet[2]] - V[tet[3]]);
        body.W[i] = glm::determinant(D_m) / 6.0;
        if (body.W[i] < 0) {
            body.W[i] = -body.W[i];
            std::swap(tet[2], tet[3]);
            D_m = glm::tmat3x3<real>(V[tet[0]] - V[tet[3]], V[tet[1]] - V[tet[3]], V[tet[2]] - V[tet[3]]);
        }
        body.B_m[i] = glm::inverse(D_m);
        glm::tmat3x3<real> D_i = glm::transpose(body.B_m[i]);
        body.D[i][0] = D_i[0];
        body.D[i][1] = D_i[1];
        body.D[i][2] = D_i[2];
        body.D[i][3] = -D_i[0] - D_i[1] - D_i[2];
    }

    using namespace Eigen;

    // Calculate mass matrix blocks

    tetrahedral_mesh_mass_matrix(body.vertices.size(), body.props.density,
                                 body.tetrahedrons.data(), body.tetrahedrons.size(), body.W.data(),
                                 body.M);

    int N_f = body.N_f; int N_c = body.N_c; int N_r = body.N_r;

    VectorXi M_ff_nz_count = VectorXi::Zero(3*N_f);
    VectorXi M_fc_nz_count = VectorXi::Zero(3*N_c);
    VectorXi M_cc_nz_count = VectorXi::Zero(3*N_c);

    for (int k = 0; k < body.M.outerSize(); ++k) {
        for (SparseMatrix<real>::InnerIterator it(body.M, k); it; ++it) {
            int row = it.row(), col = it.col();
            if (row > col) continue;
            if (row < 3*N_f && col < 3*N_f) {
                M_ff_nz_count(col)++;
            }
            else if (row < 3*N_f && col > 3*N_f) {
                M_fc_nz_count(col-3*N_f)++;
            }
            else {
                M_cc_nz_count(col-3*N_f)++;
            }
        }
    }

    body.M_ff = SparseMatrix<real>(3*N_f, 3*N_f);
    body.M_fc = SparseMatrix<real>(3*N_f, 3*N_c);
    body.M_cc = SparseMatrix<real>(3*N_c, 3*N_c);
    body.M_ff.reserve(M_ff_nz_count);
    body.M_fc.reserve(M_fc_nz_count);
    body.M_cc.reserve(M_cc_nz_count);

    // Calculate system matrix A_ff used for ADMM
    // (A_fr and A_rr are obtained while running simulation)
    for (int k = 0; k < body.M.outerSize(); ++k) {
        for (SparseMatrix<real>::InnerIterator it(body.M, k); it; ++it) {
            int row = it.row(), col = it.col();
            if (row > col) continue;
            if (row < 3*N_f && col < 3*N_f) {
                body.M_ff.insert(row, col) = it.value();
            }
            else if (row < 3*N_f && col >= 3*N_f) {
                body.M_fc.insert(row, col-3*N_f) = it.value();
            }
            else {
                body.M_cc.insert(row-3*N_f, col-3*N_f) = it.value();
            }
        }
    }
    body.A_ff = body.M_ff;

#define X(CTYPE, CFIELD) \
    for (const auto& c : constraints.CFIELD) { \
        auto& D = body.D[c.tet_id]; \
        glm::ivec4 tet = body.tetrahedrons[c.tet_id]; \
        for (int j = 0; j < 4; j++) { \
            for (int k = 0; k < 4; k++) { \
                real dA = dt * dt * c.k * body.W[c.tet_id] * glm::dot(D[j], D[k]); \
                if (tet[j] < N_f && tet[k] < N_f) { \
                    body.A_ff.coeffRef(3*tet[j]+0, 3*tet[k]+0) += dA; \
                    body.A_ff.coeffRef(3*tet[j]+1, 3*tet[k]+1) += dA; \
                    body.A_ff.coeffRef(3*tet[j]+2, 3*tet[k]+2) += dA; \
                } \
            } \
        } \
    }
    ADMM_VOLUME_CONSTRAINTS
#undef X

    // Pre-factorize system matrices
    body.M_LDLt.analyzePattern(body.M);
    body.M_LDLt.factorize(body.M);

    body.A_ff_LDLt.analyzePattern(body.A_ff);
    body.A_ff_LDLt.factorize(body.A_ff);
}

template <class Constraint>
void admm_dynamics_with_art_volume_local_solve(
        const SoftBodyWithArtData& body, const Constraint* constraints, uint32_t num_constraints,
        const glm::tvec3<real>* x,
        OUT glm::tmat3x3<real>* z, OUT glm::tmat3x3<real>* u,
        OUT glm::tmat3x3<real>* F, OUT glmx::SVD_mats<real>* F_svd) {

// #pragma omp parallel for schedule(static)
    for (int cidx = 0; cidx < num_constraints; cidx++) {
        auto& c = constraints[cidx];
        glm::ivec4 tet = body.tetrahedrons[c.tet_id];
        auto D_x = glm::rmat3(x[tet[0]] - x[tet[3]], x[tet[1]] - x[tet[3]], x[tet[2]] - x[tet[3]]) * body.B_m[c.tet_id];
        F[c.tet_id] = D_x + u[c.tet_id];
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
void admm_dynamics_with_art_update_A_constrained(
        const SoftBodyWithArtData& body, const Constraint* constraints, uint32_t num_constraints, real dt,
        const Matrix<real, Dynamic, Dynamic>& J_cr,
        INOUT Matrix<real, Dynamic, Dynamic>& A_fr, INOUT Matrix<real, Dynamic, Dynamic>& A_rr) {

    using Matrix3r = Matrix<real, 3, 3>;

    int N_f = body.N_f;
    for (int cidx = 0; cidx < num_constraints; cidx++) {
        auto& c = constraints[cidx];
        auto& D = body.D[c.tet_id];
        glm::ivec4 tet = body.tetrahedrons[c.tet_id];
        for (int j = 0; j < 4; j++) {
            for (int k = 0; k < 4; k++) {
                if (tet[j] > tet[k] || (tet[j] < N_f && tet[k] < N_f)) {
                    continue;
                }
                real D_jk = glm::dot(D[j], D[k]);
                real dA = dt * dt * c.k * body.W[c.tet_id] * D_jk;
                Matrix<real, 3, Dynamic> J_k = J_cr.middleRows<3>(3*(tet[k]-N_f));
                if (tet[j] < N_f && tet[k] >= N_f) {
                    A_fr.row(3*tet[j]+0) += dA * J_k.row(0);
                    A_fr.row(3*tet[j]+1) += dA * J_k.row(1);
                    A_fr.row(3*tet[j]+2) += dA * J_k.row(2);
                }
                else if (tet[j] >= N_f && tet[k] >= N_f) {
                    Matrix<real, 3, Dynamic> J_j = J_cr.middleRows<3>(3*(tet[j]-N_f));
                    A_rr += dA * J_j.transpose() * J_k;
                }
            }
        }
    }
}

template <class Constraint>
void admm_dynamics_with_art_update_b(
        const SoftBodyWithArtData& body, const Constraint* constraints, uint32_t num_constraints,
        real dt, const glm::tmat3x3<real>* z, const glm::tmat3x3<real>* u, const glm::tvec3<real>* x0,
        const MatrixXr& J_cr,
        INOUT real* b) {
    for (int cidx = 0; cidx < num_constraints; cidx++) {
        auto& c = constraints[cidx];
        glm::ivec4 tet = body.tetrahedrons[c.tet_id];
        glm::rmat3 p = z[c.tet_id] - u[c.tet_id];
        auto& D_i = body.D[c.tet_id];
        auto D_x0 = glm::rmat3(x0[tet[0]] - x0[tet[3]], x0[tet[1]] - x0[tet[3]], x0[tet[2]] - x0[tet[3]]) * body.B_m[c.tet_id];
        real k_s = dt * c.k * body.W[c.tet_id];
        for (int j = 0; j < 4; j++) {
            glm::tvec3<real> db = k_s * ((p - D_x0) * D_i[j]);
            if (tet[j] < body.N_f) {
                b[3*tet[j]+0] += db[0];
                b[3*tet[j]+1] += db[1];
                b[3*tet[j]+2] += db[2];
            }
            else {
                Matrix<real, 3, Dynamic> J_j = J_cr.middleRows<3>(3*(tet[j]-body.N_f));
                Map<Matrix<real, 3, 1>> db_eigen((real*)&db);
                VectorXr db_r = J_j.transpose() * db_eigen;
                Map<VectorXr> b_r_eigen(b + 3*body.N_f, body.N_r);
                b_r_eigen += db_r;
            }
        }
    }
}

// TODO: Need to fix local updates on soft body not working
void admm_dynamics_with_art(const SoftBodyWithArtData& data, const ADMMConstraints& constraints,
                            real dt, const real* sb_f, const real* art_f,
                            INOUT real* sb_pos, INOUT real* sb_vel,
                            INOUT real* art_pos, INOUT real* art_vel) {

    auto& art = data.art;
    int num_art_links = art.get_num_joints();
    int num_art_pos_dofs = art.get_num_pos_dofs();
    int N_s = data.N_s, N_f = data.N_f, N_c = data.N_c, N_r = data.N_r;

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
    /*
    MatrixXr M_r_inv(N_r, N_r);
    dynmat_view<real> M_r_inv_view(M_r_inv.data(), N_r, N_r);
    dynmat<real> identity(num_art_vel_dofs, IDENTITY);
    multiply_inverse_mass_matrix(art, dt, art_pos, identity.to_view(), OUT M_r_inv_view);
     */

    Map<VectorXr> x_s(sb_pos, 3*N_s);
    Map<VectorXr> x_f(sb_pos, 3*N_f);
    Map<VectorXr> x_c(sb_pos + 3*N_f, 3*N_c);
    Map<VectorXr> x_r(art_pos, num_art_pos_dofs);

    Map<VectorXr> v_s(sb_vel, 3*N_s);
    Map<VectorXr> v_f(sb_vel, 3*N_f);
    Map<VectorXr> v_c(sb_vel + 3*N_f, 3*N_c);
    Map<VectorXr> v_r(art_vel, N_r);

    Map<const VectorXr> f_s_ext(sb_f, 3*N_s);
    Map<const VectorXr> f_r_ext(art_f, N_r);

    VectorXr x_s_orig = x_s;
    VectorXr v_s_tilde = v_s + dt * data.M_LDLt.solve(f_s_ext);
    Map<VectorXr> v_f_tilde(v_s_tilde.data(), 3*N_f);
    Map<VectorXr> v_c_tilde(v_s_tilde.data() + 3*N_f, 3*N_c);

    VectorXr x_r_orig = x_r;
    VectorXr v_r_dot(N_r);
    featherstone_forward_dynamics(art, glm::rvec3(0), dt, nullptr, art_pos, art_vel, art_f, OUT v_r_dot.data());
    VectorXr v_r_tilde = v_r + dt * v_r_dot;

    v_c_tilde = J_cr * v_r_tilde;

    // Precalculate system matrices A_fr and A_rr
    MatrixXr A_fr = data.M_fc * J_cr;
    MatrixXr A_rr = M_r + J_cr.transpose() * data.M_cc * J_cr;

#define X(CTYPE, CFIELD) \
    admm_dynamics_with_art_update_A_constrained(data, constraints.CFIELD.data(), constraints.CFIELD.size(), dt, \
        J_cr, INOUT A_fr, INOUT A_rr);
    ADMM_VOLUME_CONSTRAINTS
#undef X

    // Precalculate system vector b0
    VectorXr b0(3*N_f + N_r);
    b0.topRows(3*N_f) = (data.M * v_s_tilde).topRows(3*N_f);
    b0.bottomRows(N_r) = M_r * v_r_tilde
                         + (data.M_fc * J_cr).transpose() * v_f_tilde
                         + (data.M_cc * J_cr).transpose() * v_c_tilde;

    // Set initial values
    v_s = v_s_tilde;
    v_r = v_r_tilde;
    x_s = x_s_orig + dt*v_s;
    integrate_implicit_euler(art, dt, nullptr, x_r.data(), v_r.data());

    // Other temporary variables used for ADMM
    std::vector<glm::tmat3x3<real>> u(data.tetrahedrons.size(), glm::tmat3x3<real>(0.0));
    std::vector<glm::tmat3x3<real>> z(data.tetrahedrons.size());
    std::vector<glm::tmat3x3<real>> p(data.tetrahedrons.size());
    std::vector<glm::tmat3x3<real>> F(data.tetrahedrons.size());
    std::vector<glmx::SVD_mats<real>> F_svd(data.tetrahedrons.size());

    std::cout << "Starting ADMM loop" << std::endl;
    for (int iter = 0; iter < 10; iter++) {

        // Local solve
#define X(CTYPE, CFIELD) \
        admm_dynamics_with_art_volume_local_solve(data, constraints.CFIELD.data(), constraints.CFIELD.size(), \
            (glm::rvec3*)x_s.data(), OUT z.data(), OUT u.data(), OUT F.data(), OUT F_svd.data());
        ADMM_VOLUME_CONSTRAINTS
#undef X

        // Global solve
        VectorXr b = b0;
        Map<VectorXr> b_f(b.data(), 3*N_f);
        Map<VectorXr> b_r(b.data() + 3*N_f, N_r);

#define X(CTYPE, CFIELD) \
        admm_dynamics_with_art_update_b(data, constraints.CFIELD.data(), constraints.CFIELD.size(), \
            dt, z.data(), u.data(), (glm::rvec3*)x_s_orig.data(), J_cr, OUT b.data());
        ADMM_VOLUME_CONSTRAINTS
#undef X
        // std::cout << "b: " << b.transpose() << std::endl;

        // Solve system using Schur complement
        MatrixXr A_comp = A_rr - A_fr.transpose() * data.A_ff_LDLt.solve(A_fr);
        VectorXr b_comp = b_r - A_fr.transpose() * data.A_ff_LDLt.solve(b_f);
        v_r = A_comp.bdcSvd(ComputeThinU | ComputeThinV).solve(b_comp); // TODO: is LDLT good enough?
        v_f = data.A_ff_LDLt.solve(b_f - A_fr*v_r);
        v_c = J_cr * v_r;

        x_s = x_s_orig + dt*v_s;
        integrate_implicit_euler(art, dt, nullptr, x_r_orig.data(), v_r.data());
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

    // TODO: Remove this projection step
    // Project constrained velocities to articulation
    /*
    v_c = J_cr * v_r;
    x_s = x_s_orig + dt*v_s;
    integrate_implicit_euler(art, dt, nullptr, x_r_orig.data(), v_r.data());
     */

    // Project constrained positions to articulation
    /*
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
     */
}

}