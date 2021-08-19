//
// Created by lasagnaphil on 21. 3. 12..
//

#include "artsim/art_with_soft_bodies.h"
#include "artsim/art_dynamics.h"
#include "artsim/math/fastsvd.h"
#include "artsim/tet_mesh.h"
#include "artsim/utils/xml.h"

#include <iostream>
#include <filesystem>
#include <tinyxml2.h>
#include <glm/gtc/type_ptr.hpp>

using namespace glmx;
using namespace tinyxml2;
namespace fs = std::filesystem;

namespace artsim {

std::vector<double> split_to_double(const std::string& input, int num)
{
    std::vector<double> result;
    std::string::size_type sz = 0, nsz = 0;
    for(int i = 0; i < num; i++){
        result.push_back(std::stof(input.substr(sz), &nsz));
        sz += nsz;
    }
    return result;
}

glm::tvec1<real> string_to_vector1d(const std::string& input) {
    std::vector<double> v = split_to_double(input, 1);
    return glm::tvec1<real>(v[0]);
}

glm::tvec3<real> string_to_vector3d(const std::string& input) {
    std::vector<double> v = split_to_double(input, 3);
    return {v[0], v[1], v[2]};
}

glm::tvec4<real> string_to_vector4d(const std::string& input) {
    std::vector<double> v = split_to_double(input, 4);
    return {v[0], v[1], v[2], v[3]};
}

glm::tmat3x3<real> string_to_matrix3d(const std::string& input) {
    std::vector<double> v = split_to_double(input, 9);
    auto M = glm::transpose(glm::make_mat3x3(v.data()));
    return M;
}

void ArtWithSoftBodies::load(const char* metadata) {
    fs::path metadata_path(metadata);
    fs::path folder = metadata_path.parent_path();

    doc = std::make_shared<XMLDocument>();
    doc->LoadFile(metadata);
    auto root_el = doc->RootElement();

    auto sim_el = root_el->FirstChildElement("simulation");
    int hz = sim_el->IntAttribute("hz");
    dt = 1.0 / hz;
    gravity = string_to_vector3d(sim_el->Attribute("gravity"));

    auto articulation_el = root_el->FirstChildElement("articulation");
    ArticulatedBodySpec art_spec;
    bool art_loaded = artsim::load_from_xml(articulation_el, folder.string().c_str(), OUT art_spec);
    if (!art_loaded) {
        exit(EXIT_FAILURE);
    }
    art.init(art_spec);

    int sb_count = 0;
    for (XMLElement* sb_el = root_el->FirstChildElement("soft_body");
         sb_el != nullptr; sb_el = sb_el->NextSiblingElement("soft_body")) { sb_count++; }

    soft_bodies = std::vector<SoftBody>(sb_count);
    sb_constraints.resize(sb_count);
    sb_names.resize(sb_count);

    sb_vert_start_idx.resize(sb_count + 1);
    sb_vert_start_idx[0] = 0;
    sb_tet_start_idx.resize(sb_count + 1);
    sb_tet_start_idx[0] = 0;

    soft_body_props.resize(sb_count);

    int sb_idx = 0;
    for (XMLElement* sb_el = root_el->FirstChildElement("soft_body"); sb_el != nullptr; sb_el = sb_el->NextSiblingElement("soft_body")) {
        fs::path soft_body_file = folder / sb_el->Attribute("file");
        TetMesh tet_mesh;
        if (soft_body_file.extension() == ".msh") {
            tet_mesh.load_msh(soft_body_file.c_str());
        }
        else {
            fprintf(stderr, "Invalid extension name for soft body mesh!\n");
            exit(EXIT_FAILURE);
        }

        SoftBodyProperties props;

        auto mat_el = sb_el->FirstChildElement("material");
        std::string mat_type = mat_el->Attribute("type");
        props.young_modulus = mat_el->DoubleAttribute("young_modulus");
        props.poisson_ratio = mat_el->DoubleAttribute("poisson_ratio");
        props.density = mat_el->DoubleAttribute("density");

        soft_bodies[sb_idx].load(tet_mesh);
        auto& sb = soft_bodies[sb_idx];
        int sb_num_vertices = sb.verts.size();
        int sb_num_tets = sb.tets.size();

        if (mat_el->NoChildren()) {
            // Material is applied to entire soft body
            real mu = props.calc_mu();
            real lambda = props.calc_lambda();
            if (mat_type == "arap") {
                real k = props.calc_arap_stiffness();
                int sb_num_tets = sb.tets.size();
                for (int i = 0; i < sb_num_tets; i++) {
                    sb_constraints[sb_idx].arap_energy.push_back({i, k, mu});
                }
            }
            if (mat_type == "corotational") {
                real k = props.calc_corotational_stiffness();
                int sb_num_tets = sb.tets.size();
                for (int i = 0; i < sb_num_tets; i++) {
                    sb_constraints[sb_idx].corotational_energy.push_back({i, k, mu, lambda});
                }
            }
            else if (mat_type == "neohookean") {
                real k = props.calc_neohookean_stiffness();
                int sb_num_tets = sb.tets.size();
                for (int i = 0; i < sb_num_tets; i++) {
                    sb_constraints[sb_idx].neohookean_energy.push_back({i, k, mu, lambda});
                }
            }
        }
        else {
            fprintf(stderr, "Unimplemented!\n");
            exit(EXIT_FAILURE);
        }

        soft_body_props[sb_idx] = props;
        sb_names[sb_idx] = soft_body_file.stem().string();

        sb_idx++;
        sb_vert_start_idx[sb_idx] = sb_vert_start_idx[sb_idx-1] + sb_num_vertices;
        sb_tet_start_idx[sb_idx] = sb_tet_start_idx[sb_idx-1] + sb_num_tets;
    }

#pragma omp parallel for
    for (int sb_idx = 0; sb_idx < sb_count; sb_idx++) {
        auto& sb = soft_bodies[sb_idx];
        auto& constraints = sb_constraints[sb_idx];
        sb.build_mass(soft_body_props[sb_idx].density, dt);
        for (auto& c : constraints.arap_energy) {
            sb.add_volume_constraint(c);
        }
        for (auto& c : constraints.corotational_energy) {
            sb.add_volume_constraint(c);
        }
        for (auto& c : constraints.neohookean_energy) {
            sb.add_volume_constraint(c);
        }
        for (auto& c : constraints.positional) {
            sb.add_positional_constraint(c);
        }
        sb.factorize();
    }

    N_s = sb_vert_start_idx[sb_count];

    sb_constraints.resize(sb_count);
    sb_constr_vertices.resize(sb_count);

    sb_idx = 0;
    for (XMLElement* sb_el = root_el->FirstChildElement("soft_body"); sb_el != nullptr; sb_el = sb_el->NextSiblingElement("soft_body")) {
        for (auto at_el = sb_el->FirstChildElement("attachment"); at_el != nullptr; at_el = at_el->NextSiblingElement("attachment")) {
            std::string node_name = at_el->Attribute("node");
            int link_idx = art.get_spec().get_index(node_name.c_str());
            if (link_idx == -1) {
                fprintf(stderr, "Cannot find node name %s for attachment!\n", node_name.c_str());
                exit(EXIT_FAILURE);
            }
            std::vector<int> constr_vertices;
            for (auto vertices_el = at_el->FirstChildElement("vertices"); vertices_el != nullptr; vertices_el = vertices_el->NextSiblingElement("vertices")) {
                std::stringstream ss(vertices_el->GetText());
                std::string token;
                while (ss >> token) {
                    int idx = std::stoi(token);
                    constr_vertices.push_back(idx);
                }
            }

            sb_constr_vertices[sb_idx].insert({link_idx, constr_vertices});
        }
        sb_idx++;
    }

    update_attachments();
}

void ArtWithSoftBodies::update_attachments() {
    index_s_to_c.resize(N_s, -1);
    index_c_to_link.resize(N_s, -1);
    index_link_to_sb.resize(art.get_num_links());

    int cur_cidx = 0;
    for (int sb_idx = 0; sb_idx < soft_bodies.size(); sb_idx++) {
        int vidx_start = sb_vert_start_idx[sb_idx];
        for (auto& [link_idx, constr_vertices] : sb_constr_vertices[sb_idx]) {
            index_link_to_sb[link_idx].push_back(sb_idx);
            for (auto& idx : constr_vertices) {
                int vidx = vidx_start + idx;
                index_s_to_c[vidx] = cur_cidx;
                index_c_to_link[cur_cidx] = link_idx;
                cur_cidx++;
            }
        }
    }

    N_c = cur_cidx;
    index_c_to_link.resize(N_c);

    N_f = N_s - N_c;
    index_c_to_s.resize(N_c);
    for (int i = 0; i < N_s; i++) {
        int cidx = index_s_to_c[i];
        if (cidx != -1) {
            index_c_to_s[cidx] = i;
        }
    }

    N_r = art.get_num_vel_dofs();

    N_t = 0;
    for (int sb_idx = 0; sb_idx < soft_bodies.size(); sb_idx++) {
        N_t += soft_bodies[sb_idx].tets.size();
    }

    x_s.resize(3*N_s);
    v_s.resize(3*N_s);
    f_s.resize(3*N_s);

    J_cr.resize(3*N_c, N_r);
    M_r.resize(N_r, N_r);
    M_r_inv.resize(N_r, N_r);
    M_r_inv_J_cr_T.resize(N_r, 3*N_c);

    reset();

    art.forward_kinematics();
    constr_vertices_offset.resize(N_c);
    for (int cidx = 0; cidx < N_c; cidx++) {
        int vidx = index_c_to_s[cidx];
        int link_idx = index_c_to_link[cidx];
        rvec3 vpos = glm::make_vec3(x_s.data() + 3*vidx);
        constr_vertices_offset[cidx] = ttransform<real>(vpos) / art.get_global_joint_trans(link_idx);
    }

    printf("Total vertices: %d\n", N_s);
    printf("Constrained vertices: %d\n", N_c);
}

void ArtWithSoftBodies::save(const char* metadata) {
    auto root_el = doc->RootElement();

    // Update attachments in XML file
    int sb_idx = 0;
    for (XMLElement* sb_el = root_el->FirstChildElement("soft_body"); sb_el != nullptr; sb_el = sb_el->NextSiblingElement("soft_body")) {
        for (auto at_el = sb_el->FirstChildElement("attachment"); at_el != nullptr; at_el = at_el->NextSiblingElement("attachment")) {
            auto node_name = at_el->Attribute("node");
            int link_idx = art.get_spec().get_index(node_name);
            if (link_idx == -1) {
                fprintf(stderr, "Cannot find node name %s for attachment!\n", node_name);
                exit(EXIT_FAILURE);
            }
            auto& vertices = sb_constr_vertices[sb_idx][link_idx];
            auto vertices_el = at_el->FirstChildElement("vertices");
            std::stringstream ss;
            for (int idx : vertices) {
                ss << std::to_string(idx) << " ";
            }
            std::string str = ss.str();
            std::cout << str << std::endl;
            vertices_el->SetText(str.c_str());
        }
        sb_idx++;
    }

    doc->SaveFile(metadata);
}

void ArtWithSoftBodies::reset() {
    for (int sb_idx = 0; sb_idx < soft_bodies.size(); sb_idx++) {
        auto& sb = soft_bodies[sb_idx];
        real* vertices_ptr = (real*) sb.verts.data();
        std::copy(vertices_ptr, vertices_ptr + 3*sb.verts.size(), x_s.data() + 3 * sb_vert_start_idx[sb_idx]);
    }
    v_s.setZero();
    f_s.setZero();

    art.reset();
}


template <class Constraint>
void admm_vel_volume_constraint_local_solve(
        const SoftBody& body, const Constraint* constraints, uint32_t num_constraints,
        const glm::tmat3x3<real>* F, const glmx::SVD_mats<real>* F_svd,
        OUT glm::tmat3x3<real>* z, OUT glm::tmat3x3<real>* u) {

    for (int cidx = 0; cidx < num_constraints; cidx++) {
        auto& c = constraints[cidx];
        auto& svd = F_svd[c.tet_id];
        glm::rvec3 sigma = proximal_eigvec(svd.Sigma, body.W[c.tet_id], c);
        z[c.tet_id] = glmx::svd_mult(svd.U, sigma, svd.V);
        u[c.tet_id] = F[c.tet_id] - z[c.tet_id];
    }
}

#define X(CTYPE, CFIELD) \
template void admm_vel_volume_constraint_local_solve( \
        const SoftBody&, const CTYPE*, uint32_t, \
        const glm::tmat3x3<real>* F, const glmx::SVD_mats<real>* F_svd, \
        OUT glm::tmat3x3<real>* z, OUT glm::tmat3x3<real>* u);
ADMM_VOLUME_CONSTRAINTS
#undef X

template <class Constraint>
void admm_vel_volume_constraint_update_b(
        const SoftBody& body, const Constraint* constraints, uint32_t num_constraints,
        real dt, const glm::tmat3x3<real>* z, const glm::tmat3x3<real>* u, const glm::tvec3<real>* x0,
        INOUT real* b) {

    for (int cidx = 0; cidx < num_constraints; cidx++) {
        auto& c = constraints[cidx];
        glm::ivec4 tet = body.tets[c.tet_id];
        auto p = z[c.tet_id] - u[c.tet_id];
        auto& D_i = body.D[c.tet_id];
        auto D_x0 = glm::rmat3(x0[tet[0]] - x0[tet[3]], x0[tet[1]] - x0[tet[3]], x0[tet[2]] - x0[tet[3]]) * body.B_m[c.tet_id];
        real k_s = dt * c.k * body.W[c.tet_id];
        for (int j = 0; j < 4; j++) {
            glm::tvec3<real> db = k_s * ((p - D_x0) * D_i[j]);
            b[3*tet[j]+0] += db[0];
            b[3*tet[j]+1] += db[1];
            b[3*tet[j]+2] += db[2];
        }
    }
}

#define X(CTYPE, CFIELD) \
template void admm_vel_volume_constraint_update_b( \
        const SoftBody& body, const CTYPE* constraints, uint32_t num_constraints, \
        real dt, const glm::tmat3x3<real>* z, const glm::tmat3x3<real>* u, const glm::tvec3<real>* x0, INOUT real* b);
ADMM_VOLUME_CONSTRAINTS
#undef X


void ArtWithSoftBodies::admm_calc_deformation_field_and_svd(
        const glm::tmat3x3<real>* u, OUT glm::tmat3x3<real>* F, OUT glmx::SVD_mats<real>* F_svd) {
    for (int sb_idx = 0; sb_idx < soft_bodies.size(); sb_idx++) {
        auto& sb = soft_bodies[sb_idx];
        int start_vidx = sb_vert_start_idx[sb_idx];
        int start_tidx = sb_tet_start_idx[sb_idx];
        soft_body_calc_deformation_field(sb, (glm::rvec3*)x_s.data() + 3*start_vidx, u,
                                         OUT F + start_tidx);
    }
    glmx::fastsvd(F, N_t, F_svd);
}

void ArtWithSoftBodies::admm_local_solve(
        const glm::tmat3x3<real>* F, const glmx::SVD_mats<real>* F_svd,
        OUT glm::tmat3x3<real>* z, OUT glm::tmat3x3<real>* u) {

    for (int sb_idx = 0; sb_idx < soft_bodies.size(); sb_idx++) {
        auto& sb = soft_bodies[sb_idx];
        auto& constraints = sb_constraints[sb_idx];
        int start_tidx = sb_tet_start_idx[sb_idx];
#define X(CTYPE, CFIELD) \
        admm_vel_volume_constraint_local_solve( \
                soft_bodies[sb_idx], \
                constraints.CFIELD.data(), \
                constraints.CFIELD.size(), \
                F + start_tidx, F_svd + start_tidx, \
                OUT z + start_tidx, OUT u + start_tidx);
        ADMM_VOLUME_CONSTRAINTS
#undef X
    }
}

void ArtWithSoftBodies::admm_update_b(
        real dt, const glm::tmat3x3<real>* z, const glm::tmat3x3<real>* u, const glm::tvec3<real>* x0,
        INOUT real* b) {

    for (int sb_idx = 0; sb_idx < soft_bodies.size(); sb_idx++) {
        auto& sb = soft_bodies[sb_idx];
        auto& constraints = sb_constraints[sb_idx];
        int start_vidx = sb_vert_start_idx[sb_idx];
        int start_tidx = sb_tet_start_idx[sb_idx];
#define X(CTYPE, CFIELD) \
        admm_vel_volume_constraint_update_b( \
                soft_bodies[sb_idx], \
                constraints.CFIELD.data(), \
                constraints.CFIELD.size(), \
                dt, z + start_tidx, u + start_tidx, x0 + start_tidx, b + start_vidx);
        ADMM_VOLUME_CONSTRAINTS
#undef X
    }
}

void ArtWithSoftBodies::admm_update_residuals(
        const glm::tmat3x3<real>* z_prev, const glm::tmat3x3<real>* z_next, const glm::tvec3<real>* x,
        INOUT real& primal_res_sq, INOUT real& dual_res_sq) {

    for (int sb_idx = 0; sb_idx < soft_bodies.size(); sb_idx++) {
        auto& sb = soft_bodies[sb_idx];
        auto& constraints = sb_constraints[sb_idx];
        int start_vidx = sb_vert_start_idx[sb_idx];
        int start_tidx = sb_tet_start_idx[sb_idx];
#define X(CTYPE, CFIELD) \
        admm_volume_constraint_update_residuals( \
                soft_bodies[sb_idx], \
                constraints.CFIELD.data(), \
                constraints.CFIELD.size(), \
                z_prev + start_tidx, z_next + start_tidx, x + start_tidx, primal_res_sq, dual_res_sq);
        ADMM_VOLUME_CONSTRAINTS
#undef X
    }
}

void ArtWithSoftBodies::apply_selector_matrix(const real* X_s, OUT real* X_c) {
    for (int cidx = 0; cidx < N_c; cidx++) {
        int vidx = index_c_to_s[cidx];
        X_c[3*cidx+0] = X_s[3*vidx+0];
        X_c[3*cidx+1] = X_s[3*vidx+1];
        X_c[3*cidx+2] = X_s[3*vidx+2];
    }
}

void ArtWithSoftBodies::apply_selector_matrix_inv(const real* X_c, OUT real* X_s) {
    for (int vidx = 0; vidx < N_s; vidx++) {
        int cidx = index_s_to_c[vidx];
        if (cidx == -1) {
            X_s[3*vidx+0] = 0;
            X_s[3*vidx+1] = 0;
            X_s[3*vidx+2] = 0;
        }
        else {
            X_s[3*vidx+0] = X_c[3*cidx+0];
            X_s[3*vidx+1] = X_c[3*cidx+1];
            X_s[3*vidx+2] = X_c[3*cidx+2];
        }
    }
}

void ArtWithSoftBodies::apply_selector_matrix_add(INOUT real* X_s, const real* dX_c) {
    for (int cidx = 0; cidx < N_c; cidx ++) {
        int vidx = index_c_to_s[cidx];
        X_s[3*vidx+0] += dX_c[3*cidx+0];
        X_s[3*vidx+1] += dX_c[3*cidx+1];
        X_s[3*vidx+2] += dX_c[3*cidx+2];
    }
}

void ArtWithSoftBodies::apply_selector_matrix_sub(INOUT real* X_s, const real* dX_c) {
    for (int cidx = 0; cidx < N_c; cidx ++) {
        int vidx = index_c_to_s[cidx];
        X_s[3*vidx+0] -= dX_c[3*cidx+0];
        X_s[3*vidx+1] -= dX_c[3*cidx+1];
        X_s[3*vidx+2] -= dX_c[3*cidx+2];
    }
}

void ArtWithSoftBodies::calc_constraint_jacobian() {
    J_cr.setZero();
    auto& art_spec = art.get_spec();
    for (int cidx = 0; cidx < N_c; cidx++) {
        int vidx = index_c_to_s[cidx];
        int link_idx = index_c_to_link[cidx];
        rvec3 vpos = glm::make_vec3(x_s.data() + 3*vidx);

        int lidx = link_idx;
        while (lidx != -1) {
            int joint_vel_dof_start = art_spec.joint_vel_dof_starts[lidx];
            int joint_vel_dofs = art_spec.joint_vel_dofs[lidx];
            auto& joint = art_spec.joints[lidx];
            auto T_v = art.get_global_joint_trans(link_idx) * constr_vertices_offset[cidx];
            auto T = art.get_global_joint_trans(lidx) / T_v;
            glm::rvec3* J = (glm::rvec3*)&J_cr(3*cidx, joint_vel_dof_start);
            switch (joint.type) {
                case JOINT_TYPE_REVOLUTE_X:  J[0] = T_v.R * Ad(T, rscrew(1, 0, 0, 0, 0, 0)).v; break;
                case JOINT_TYPE_REVOLUTE_Y:  J[0] = T_v.R * Ad(T, rscrew(0, 1, 0, 0, 0, 0)).v; break;
                case JOINT_TYPE_REVOLUTE_Z:  J[0] = T_v.R * Ad(T, rscrew(0, 0, 1, 0, 0, 0)).v; break;
                case JOINT_TYPE_PRISMATIC_X: J[0] = T_v.R * Ad(T, rscrew(0, 0, 0, 1, 0, 0)).v; break;
                case JOINT_TYPE_PRISMATIC_Y: J[0] = T_v.R * Ad(T, rscrew(0, 0, 0, 0, 1, 0)).v; break;
                case JOINT_TYPE_PRISMATIC_Z: J[0] = T_v.R * Ad(T, rscrew(0, 0, 0, 0, 0, 1)).v; break;
                case JOINT_TYPE_SPHERICAL: {
                    J[0] = T_v.R * Ad(T, rscrew(1, 0, 0, 0, 0, 0)).v;
                    J[1] = T_v.R * Ad(T, rscrew(0, 1, 0, 0, 0, 0)).v;
                    J[2] = T_v.R * Ad(T, rscrew(0, 0, 1, 0, 0, 0)).v;
                } break;
                case JOINT_TYPE_FLOATING: {
                    J[0] = T_v.R * Ad(T, rscrew(1, 0, 0, 0, 0, 0)).v;
                    J[1] = T_v.R * Ad(T, rscrew(0, 1, 0, 0, 0, 0)).v;
                    J[2] = T_v.R * Ad(T, rscrew(0, 0, 1, 0, 0, 0)).v;
                    J[3] = T_v.R * Ad(T, rscrew(0, 0, 0, 1, 0, 0)).v;
                    J[4] = T_v.R * Ad(T, rscrew(0, 0, 0, 0, 1, 0)).v;
                    J[5] = T_v.R * Ad(T, rscrew(0, 0, 0, 0, 0, 1)).v;
                } break;
            }
            lidx = art_spec.parents[lidx];
        }
    }
}

VectorXr ArtWithSoftBodies::calc_total_force_with_gravity() {
    VectorXr f_s_tot = f_s;
    auto f_s_tot_ptr = (glm::rvec3*) f_s_tot.data();
    for (int sb_idx = 0; sb_idx < soft_bodies.size(); sb_idx++) {
        int vidx_start = sb_vert_start_idx[sb_idx];
        auto& sb = soft_bodies[sb_idx];
        for (int t = 0; t < sb.tets.size(); t++) {
            glm::ivec4 tet = sb.tets[t];
            glm::rvec3 f_g = real(1. / 4.) * soft_body_props[sb_idx].density * sb.W[t] * gravity;
            f_s_tot_ptr[vidx_start + tet[0]] += f_g;
            f_s_tot_ptr[vidx_start + tet[1]] += f_g;
            f_s_tot_ptr[vidx_start + tet[2]] += f_g;
            f_s_tot_ptr[vidx_start + tet[3]] += f_g;
        }
    }
    return f_s_tot;
}

void ArtWithSoftBodies::integrate_admm_coupled() {
    Eigen::Map<VectorXr> x_r(art.get_pos_buf(), art.get_num_pos_dofs());
    Eigen::Map<VectorXr> v_r(art.get_vel_buf(), N_r);
    Eigen::Map<VectorXr> v_r_dot(art.get_acc_buf(), N_r);
    Eigen::Map<VectorXr> f_r(art.get_internal_force_buf(), N_r);

    // Forward kinematics of articulation
    art.forward_kinematics();

    // Calculate coupling jacobian
    calc_constraint_jacobian();

    // Calculate articulation matrix M_r
    dynmat_view<real> M_r_view(M_r.data(), N_r, N_r);
    art.mass_matrix(OUT M_r_view, dt);

    // Calculate inverse of articulation matrix M_r^{-1}
    dynmat_view<real> M_r_inv_view(M_r_inv.data(), N_r, N_r);
    dynmat<real> identity(N_r, IDENTITY);
    art.multiply_inverse_mass_matrix(identity.to_view(), OUT M_r_inv_view, dt);

    // Calculate other matrices related to articulation
    M_r_inv_J_cr_T.noalias() = M_r_inv * J_cr.transpose();

    // Add gravity to total force
    VectorXr f_s_tot = calc_total_force_with_gravity();

    // Semi-implicit integration with explicit forces
    VectorXr x_s_orig = x_s;
    VectorXr v_s_tilde = v_s;
    for (int sb_idx = 0; sb_idx < soft_bodies.size(); sb_idx++) {
        int vidx_start = sb_vert_start_idx[sb_idx];
        int vidx_count = sb_vert_start_idx[sb_idx + 1] - vidx_start;
        auto& sb = soft_bodies[sb_idx];
        v_s_tilde.middleRows(3*vidx_start, 3*vidx_count) +=
                dt * sb.M_LDLt.solve(f_s_tot.middleRows(3*vidx_start, 3*vidx_count));
    }
    VectorXr x_r_orig = x_r;
    art.forward_dynamics(gravity, dt);
    VectorXr v_r_tilde = v_r + dt * v_r_dot;
    v_s = v_s_tilde;
    v_r = v_r_tilde;
    x_s = x_s_orig + dt*v_s;

    // ADMM optimization
    std::vector<glm::tmat3x3<real>> u(N_t, glm::tmat3x3<real>(0.0));
    std::vector<glm::tmat3x3<real>> u_prev(N_t);
    std::vector<glm::tmat3x3<real>> z(N_t, glm::tmat3x3<real>(0.0));
    std::vector<glm::tmat3x3<real>> z_prev(N_t);
    std::vector<glm::tmat3x3<real>> p(N_t);
    std::vector<glm::tmat3x3<real>> F(N_t);
    std::vector<glmx::SVD_mats<real>> F_svd(N_t);

    VectorXr v_s_prev(3*N_s);
    VectorXr v_r_prev(N_r);
    VectorXr v_c(3*N_c);

    VectorXr b_s(3*N_s);
    VectorXr b_c(3*N_c);
    VectorXr b_r(N_r);
    VectorXr f_c(3*N_c);
    VectorXr r_f(3*N_c);
    VectorXr s_f(3*N_c);
    VectorXr s_f_s(3*N_s);
    VectorXr s_v(3*N_s + N_r);
    VectorXr s_v_c(3*N_c);
    VectorXr a_f(3*N_c);

    f_c.setZero();

    real primal_res, dual_res, primal_res_prev = DBL_MAX, dual_res_prev = DBL_MAX;

    std::cout << std::endl << "Starting ADMM loop" << std::endl;
    for (int iter = 0; iter < 30; iter++) {
        // Local solve
        z_prev = z;
        u_prev = u;

        admm_calc_deformation_field_and_svd(u.data(), OUT F.data(), OUT F_svd.data());
        admm_local_solve(F.data(), F_svd.data(), OUT z.data(), OUT u.data());

        // Global solve
        v_s_prev = v_s;
        v_r_prev = v_r;

        for (int sb_idx = 0; sb_idx < soft_bodies.size(); sb_idx++) {
            int vidx_start = sb_vert_start_idx[sb_idx];
            int vidx_count = sb_vert_start_idx[sb_idx + 1] - vidx_start;
            auto& sb = soft_bodies[sb_idx];
            b_s.middleRows(3*vidx_start, 3*vidx_count) = sb.M * v_s_tilde.middleRows(3*vidx_start, 3*vidx_count);
        }
        b_r = M_r * v_r_tilde;

        admm_update_b(dt, z.data(), u.data(), (glm::rvec3*)x_s_orig.data(), OUT b_s.data());

        // f_c.setZero();

        apply_selector_matrix_sub(INOUT b_s.data(), f_c.data());
        b_r += J_cr.transpose() * f_c;

        for (int sb_idx = 0; sb_idx < soft_bodies.size(); sb_idx++) {
            int vidx_start = sb_vert_start_idx[sb_idx];
            int vidx_count = sb_vert_start_idx[sb_idx + 1] - vidx_start;
            auto& sb = soft_bodies[sb_idx];
            v_s.middleRows(3*vidx_start, 3*vidx_count) = sb.A_LDLt.solve(b_s.middleRows(3*vidx_start, 3*vidx_count));
        }
        v_r = M_r_inv * b_r;

        apply_selector_matrix(v_s.data(), OUT v_c.data());
        r_f = v_c - J_cr * v_r;
        s_f = r_f;

        int uzawa_iter = 0;
        while (r_f.squaredNorm() > 1e-4) {
            apply_selector_matrix_inv(s_f.data(), OUT s_f_s.data());
            for (int sb_idx = 0; sb_idx < soft_bodies.size(); sb_idx++) {
                int vidx_start = sb_vert_start_idx[sb_idx];
                int vidx_count = sb_vert_start_idx[sb_idx + 1] - vidx_start;
                auto& sb = soft_bodies[sb_idx];
                s_v.middleRows(3*vidx_start, 3*vidx_count) = sb.A_LDLt.solve(s_f_s.middleRows(3*vidx_start, 3*vidx_count));
            }
            s_v.bottomRows(N_r) = -M_r_inv_J_cr_T * s_f;
            apply_selector_matrix(s_v.data(), OUT s_v_c.data());
            a_f = s_v_c - J_cr * s_v.bottomRows(N_r);
            real s_f_a_f = s_f.dot(a_f);
            real alpha = s_f.dot(r_f) / s_f_a_f;
            v_s -= alpha * s_v.topRows(3*N_s);
            v_r -= alpha * s_v.bottomRows(N_r);
            f_c += alpha * s_f;
            r_f -= alpha * a_f;
            real beta = r_f.dot(a_f) / s_f_a_f;
            s_f = r_f - beta*s_f;
            uzawa_iter++;
            if (uzawa_iter == 30) break;
        }

        std::cout << "Uzawa iter converged in " << uzawa_iter << " iters! " <<
                  "(residual = " << r_f.norm() << ")" << std::endl;

        real primal_res_sq = 0, dual_res_sq = 0;
        admm_update_residuals(z_prev.data(), z.data(), (glm::rvec3*)x_s.data(), INOUT primal_res_sq, INOUT dual_res_sq);
        primal_res = sqrt(primal_res_sq);
        dual_res = sqrt(dual_res_sq);

        if (primal_res > primal_res_prev && dual_res > dual_res_prev) {
            std::cout << "primal_res = " << primal_res << ", dual_res = " << dual_res << " (abort!)" << std::endl;
            v_s = v_s_prev;
            v_r = v_r_prev;
            break;
        }
        else {
            std::cout << "primal_res = " << primal_res << ", dual_res = " << dual_res << std::endl;
            primal_res_prev = primal_res;
            dual_res_prev = dual_res;
        }

        x_s = x_s_orig + dt*v_s;
    }

    integrate_positions(art.get_spec(), dt, v_r.data(), x_r.data());

    // Project constrained positions to articulation
    art.forward_kinematics();
    for (int cidx = 0; cidx < N_c; cidx++) {
        int vidx = index_c_to_s[cidx];
        int link_idx = index_c_to_link[cidx];
        auto T = art.get_global_joint_trans(link_idx) * constr_vertices_offset[cidx];
        x_s(3*vidx+0) = T.v[0];
        x_s(3*vidx+1) = T.v[1];
        x_s(3*vidx+2) = T.v[2];
    }
}

}
