//
// Created by lasagnaphil on 21. 3. 12..
//

#include "artsim/art_with_soft_bodies.h"

#include "artsim/dynamics.h"
#include "artsim/math/fastsvd.h"
#include "artsim/tet_mesh.h"

#include <iostream>
#include <filesystem>
#include <tinyxml2.h>
#include <glm/gtc/type_ptr.hpp>

#include "tiny_obj_loader.h"

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

bool load_from_xml(XMLElement* art_elem, const fs::path& current_dir, OUT ArticulatedBody& art) {
    std::unordered_map<std::string, ttransform<real>> T_global_body_map;
    std::unordered_map<std::string, ttransform<real>> T_global_joint_map;
    std::unordered_map<std::string, int> idx_map;

    T_global_body_map["none"] = ttransform<real>(IDENTITY);
    T_global_joint_map["none"] = ttransform<real>(IDENTITY);
    idx_map["none"] = -1;

    std::string art_name = art_elem->Attribute("name");
    std::string art_xform_mode = art_elem->Attribute("xform_mode");
    if (art_xform_mode != "global") {
        std::cout << "Only xform_mode = global supported!" << std::endl;
        exit(EXIT_FAILURE);
    }

    int current_idx = 0;
    for(XMLElement* node = art_elem->FirstChildElement("node"); node != nullptr; node = node->NextSiblingElement("node"))
    {
        artsim::Joint joint;
        artsim::Link link;

        std::string name = node->Attribute("name");

        std::string parent_name = node->Attribute("parent");

        XMLElement* link_elem = node->FirstChildElement("link");

        std::string body_type = link_elem->Attribute("type");
        CollisionShape col_shape;
        RenderShape render_shape;
        if (body_type == "box") {
            glm::tvec3<real> size = string_to_vector3d(link_elem->Attribute("size"));
            col_shape = CollisionShape::make_box(size);
            render_shape = RenderShape::make_box(size);
        }
        else if (body_type == "sphere") {
            double radius = std::stod(link_elem->Attribute("radius"));
            col_shape = CollisionShape::make_sphere(radius);
            render_shape = RenderShape::make_sphere(radius);
        }
        else if (body_type == "mesh") {
            fs::path filepath = current_dir / link_elem->Attribute("obj");
            tinyobj::ObjReader reader;
            reader.ParseFromFile(filepath);
            if (reader.Valid()) {
                auto& shapes = reader.GetShapes();
                col_shape = CollisionShape::make_mesh(&reader.GetAttrib(), shapes.data(), shapes.size());
                render_shape = RenderShape::make_mesh(&reader.GetAttrib(), shapes.data(), shapes.size());
            }
            else {
                fprintf(stderr, "Invalid OBJ file %s!\n", filepath.c_str());
                fprintf(stderr, "Message: %s\n", reader.Error().c_str());
                return false;
            }
        }
        else if (body_type == "capsule") {
            double radius = std::stod(link_elem->Attribute("radius"));
            double height = std::stod(link_elem->Attribute("height"));
            printf("Capsule not supported!");
            return false;
        }

        real mass, density;
        if (link_elem->Attribute("density")) {
            density = std::stod(link_elem->Attribute("density"));
            real volume = col_shape.mass(real(1));
            mass = density * volume;
        }
        else if (link_elem->Attribute("mass")) {
            mass = std::stod(link_elem->Attribute("mass"));
            real volume = col_shape.mass(real(1));
            density = mass / volume;
        }

        tsmat3x3<real> inertia = col_shape.inertia(density);

        ttransform<real> T_global_body;
        T_global_body.R = glmx::exp_mat(string_to_vector3d(link_elem->Attribute("rot")));
        T_global_body.v = string_to_vector3d(link_elem->Attribute("pos"));

        XMLElement* joint_elem = node->FirstChildElement("joint");
        std::string joint_type = joint_elem->Attribute("type");
        real joint_damping = joint_elem->DoubleAttribute("damping");

        ttransform<real> T_global_joint;
        T_global_joint.R = glmx::exp_mat(string_to_vector3d(joint_elem->Attribute("rot")));
        T_global_joint.v = string_to_vector3d(joint_elem->Attribute("pos"));

        T_global_body_map[name] = T_global_body;
        T_global_joint_map[name] = T_global_joint;

        ttransform<real> local_joint_pose;
        if (parent_name != "none") {
            local_joint_pose = T_global_joint / T_global_joint_map[parent_name];
        }
        else {
            local_joint_pose = ttransform<real>(IDENTITY);
        }
        ttransform<real> local_link_pose = T_global_body / T_global_joint;

        link = Link::create(inertia, mass, col_shape, render_shape, local_joint_pose, local_link_pose, idx_map[parent_name], {});

        if(joint_type == "free")
        {
            // TODO: Should we also put kd on floating joints?
            joint = Joint::floating();
        }
        else if(joint_type == "ball")
        {
            joint = Joint::spherical(0, joint_damping);
        }
        else if(joint_type == "revolute")
        {
            glm::tvec3<real> axis = string_to_vector3d(joint_elem->Attribute("axis"));
            if (glm::epsilonEqual<real>(axis.x, 1.0, 1e-8)) {
                joint = Joint::revolute_x(0, joint_damping);
            }
            else if (glm::epsilonEqual<real>(axis.y, 1.0, 1e-8)) {
                joint = Joint::revolute_y(0, joint_damping);
            }
            else if (glm::epsilonEqual<real>(axis.z, 1.0, 1e-8)) {
                joint = Joint::revolute_z(0, joint_damping);
            }
            else {
                std::cout << "Only revolute joints with X, Y, or Z axis supported!" << std::endl;
                return false;
            }
        }

        art.add_link_and_joint(link, joint, name);
        idx_map[name] = current_idx;
        current_idx++;
    }

    art.setup(false);
    return true;
}

void ArtWithSoftBodies::load(const char* metadata) {
    fs::path metadata_path(metadata);
    fs::path folder = metadata_path.parent_path();

    doc = std::make_unique<XMLDocument>();
    doc->LoadFile(metadata);
    auto root_el = doc->RootElement();

    auto sim_el = root_el->FirstChildElement("simulation");
    int hz = sim_el->IntAttribute("hz");
    dt = 1.0 / hz;
    gravity = string_to_vector3d(sim_el->Attribute("gravity"));

    auto articulation_el = root_el->FirstChildElement("articulation");
    bool art_loaded = load_from_xml(articulation_el, folder, OUT art);
    if (!art_loaded) {
        exit(EXIT_FAILURE);
    }

    int sb_count = 0;
    for (XMLElement* sb_el = root_el->FirstChildElement("soft_body");
         sb_el != nullptr; sb_el = sb_el->NextSiblingElement("soft_body")) { sb_count++; }

    soft_bodies = std::vector<SoftBodyData>(sb_count);
    sb_constraints.resize(sb_count);
    sb_names.resize(sb_count);

    sb_vert_start_idx.resize(sb_count + 1);
    sb_vert_start_idx[0] = 0;
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

        soft_bodies[sb_idx].load(tet_mesh, props);
        auto& sb = soft_bodies[sb_idx];
        int sb_num_vertices = sb.vertices.size();

        if (mat_el->NoChildren()) {
            // Material is applied to entire soft body
            real mu = sb.props.calc_mu();
            real lambda = sb.props.calc_lambda();
            if (mat_type == "corotational") {
                real k = sb.props.calc_corotational_stiffness();
                int sb_num_tets = sb.tetrahedrons.size();
                for (int i = 0; i < sb_num_tets; i++) {
                    sb_constraints[sb_idx].corotational_energy.push_back({i, k, mu, lambda});
                }
            }
            else if (mat_type == "neohookean") {
                real k = sb.props.calc_neohookean_stiffness();
                int sb_num_tets = sb.tetrahedrons.size();
                for (int i = 0; i < sb_num_tets; i++) {
                    sb_constraints[sb_idx].neohookean_energy.push_back({i, k, mu, lambda});
                }
            }
        }
        else {
            fprintf(stderr, "Unimplemented!\n");
            exit(EXIT_FAILURE);
        }

        sb_names[sb_idx] = soft_body_file.stem().string();

        sb_idx++;
        sb_vert_start_idx[sb_idx] = sb_vert_start_idx[sb_idx-1] + sb_num_vertices;
    }

#pragma omp parallel for
    for (int sb_idx = 0; sb_idx < sb_count; sb_idx++) {
        soft_body_precomputation(soft_bodies[sb_idx], sb_constraints[sb_idx], dt);
    }

    N_s = sb_vert_start_idx[sb_count];

    sb_constraints.resize(sb_count);
    sb_constr_vertices.resize(sb_count);

    sb_idx = 0;
    for (XMLElement* sb_el = root_el->FirstChildElement("soft_body"); sb_el != nullptr; sb_el = sb_el->NextSiblingElement("soft_body")) {
        for (auto at_el = sb_el->FirstChildElement("attachment"); at_el != nullptr; at_el = at_el->NextSiblingElement("attachment")) {
            std::string node_name = at_el->Attribute("node");
            auto it = std::find(art.names.begin(), art.names.end(), node_name);
            if (it == art.names.end()) {
                fprintf(stderr, "Cannot find node name %s for attachment!\n", node_name.c_str());
                exit(EXIT_FAILURE);
            }
            int link_idx = it - art.names.begin();
            auto vertices_el = at_el->FirstChildElement("vertices");
            std::cout << vertices_el->GetText() << std::endl;
            std::stringstream ss(vertices_el->GetText());
            std::string token;
            std::vector<int> constr_vertices;
            while (ss >> token) {
                int idx = std::stoi(token);
                constr_vertices.push_back(idx);
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
        N_t += soft_bodies[sb_idx].tetrahedrons.size();
    }

    x_s.resize(3*N_s);
    x_r.resize(art.get_num_pos_dofs());
    v_s.resize(3*N_s);
    v_r.resize(N_r);
    f_s.resize(3*N_s);
    f_r.resize(N_r);
    art_joint_trans.resize(art.get_num_joints());
    art_link_trans.resize(art.get_num_links());
    art_joint_S.resize(N_r);

    J_cr.resize(3*N_c, N_r);
    M_r.resize(N_r, N_r);
    M_r_inv.resize(N_r, N_r);
    M_r_inv_J_cr_T.resize(N_r, 3*N_c);

    reset();

    artsim::calc_transforms(art, x_r.data(), art_joint_trans.data(), art_link_trans.data());
    constr_vertices_offset.resize(N_c);
    for (int cidx = 0; cidx < N_c; cidx++) {
        int vidx = index_c_to_s[cidx];
        int link_idx = index_c_to_link[cidx];
        rvec3 vpos = glm::make_vec3(x_s.data() + 3*vidx);
        constr_vertices_offset[cidx] = ttransform<real>(vpos) / art_joint_trans[link_idx];
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
            std::string node_name = at_el->Attribute("node");
            auto it = std::find(art.names.begin(), art.names.end(), node_name);
            if (it == art.names.end()) {
                fprintf(stderr, "Cannot find node name %s for attachment!\n", node_name.c_str());
                exit(EXIT_FAILURE);
            }
            int link_idx = it - art.names.begin();
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
        real* vertices_ptr = (real*) sb.vertices.data();
        std::copy(vertices_ptr, vertices_ptr + 3*sb.vertices.size(), x_s.data() + 3*sb_vert_start_idx[sb_idx]);
    }
    v_s.setZero();
    f_s.setZero();

    artsim::set_zero_pose(art, x_r.data());
    v_r.setZero();
    f_r.setZero();
}

void ArtWithSoftBodies::admm_local_solve(
        const glm::tvec3<real>* x,
        OUT glm::tmat3x3<real>* z, OUT glm::tmat3x3<real>* u,
        OUT glm::tmat3x3<real>* F, OUT glmx::SVD_mats<real>* F_svd) {

    int start_vidx = 0;
    int start_tidx = 0;
    for (int sb_idx = 0; sb_idx < soft_bodies.size(); sb_idx++) {
        auto& sb = soft_bodies[sb_idx];
        auto& constraints = sb_constraints[sb_idx];
#define X(CTYPE, CFIELD) \
        admm_volume_constraint_local_solve( \
                soft_bodies[sb_idx], \
                constraints.CFIELD.data(), \
                constraints.CFIELD.size(), \
                x + start_vidx, z + start_tidx, u + start_tidx, F + start_tidx, F_svd + start_tidx);
        ADMM_VOLUME_CONSTRAINTS
#undef X
        start_vidx += sb.vertices.size();
        start_tidx += sb.tetrahedrons.size();
    }
}

void ArtWithSoftBodies::admm_update_b(
        real dt, const glm::tmat3x3<real>* z, const glm::tmat3x3<real>* u, const glm::tvec3<real>* x0,
        INOUT real* b) {

    int start_vidx = 0;
    int start_tidx = 0;
    for (int sb_idx = 0; sb_idx < soft_bodies.size(); sb_idx++) {
        auto& sb = soft_bodies[sb_idx];
        auto& constraints = sb_constraints[sb_idx];
#define X(CTYPE, CFIELD) \
        admm_volume_constraint_update_b( \
                soft_bodies[sb_idx], \
                constraints.CFIELD.data(), \
                constraints.CFIELD.size(), \
                dt, z + start_tidx, u + start_tidx, x0 + start_tidx, b + start_vidx);
        ADMM_VOLUME_CONSTRAINTS
#undef X
        start_vidx += sb.vertices.size();
        start_tidx += sb.tetrahedrons.size();
    }
}

void ArtWithSoftBodies::admm_update_residuals(
        const glm::tmat3x3<real>* z_prev, const glm::tmat3x3<real>* z_next, const glm::tvec3<real>* x,
        INOUT real& primal_res_sq, INOUT real& dual_res_sq) {

    int start_tidx = 0;
    for (int sb_idx = 0; sb_idx < soft_bodies.size(); sb_idx++) {
        auto& sb = soft_bodies[sb_idx];
        auto& constraints = sb_constraints[sb_idx];
#define X(CTYPE, CFIELD) \
        admm_volume_constraint_update_residuals( \
                soft_bodies[sb_idx], \
                constraints.CFIELD.data(), \
                constraints.CFIELD.size(), \
                z_prev + start_tidx, z_next + start_tidx, x + start_tidx, primal_res_sq, dual_res_sq);
        ADMM_VOLUME_CONSTRAINTS
#undef X
        start_tidx += sb.tetrahedrons.size();
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
    calc_S(art, x_r.data(), art_joint_S.data());
    J_cr.setZero();
    for (int cidx = 0; cidx < N_c; cidx++) {
        int vidx = index_c_to_s[cidx];
        int link_idx = index_c_to_link[cidx];
        rvec3 vpos = glm::make_vec3(x_s.data() + 3*vidx);

        int lidx = link_idx;
        while (lidx != -1) {
            int joint_vel_dof_start = art.joint_vel_dof_starts[lidx];
            int joint_vel_dofs = art.joint_vel_dofs[lidx];
            for (int j = joint_vel_dof_start; j < joint_vel_dof_start + joint_vel_dofs; j++) {
                auto T_v = art_joint_trans[link_idx] * constr_vertices_offset[cidx];
                auto S_prime = Ad(art_joint_trans[lidx] / T_v, art_joint_S[j]);
                rvec3 S_v = T_v.R * S_prime.v;
                J_cr(3*cidx+0, j) = S_v[0];
                J_cr(3*cidx+1, j) = S_v[1];
                J_cr(3*cidx+2, j) = S_v[2];
            }
            lidx = art.parents[lidx];
        }
    }
}

VectorXr ArtWithSoftBodies::calc_total_force_with_gravity() {
    VectorXr f_s_tot = f_s;
    auto f_s_tot_ptr = (glm::rvec3*) f_s_tot.data();
    for (int sb_idx = 0; sb_idx < soft_bodies.size(); sb_idx++) {
        int vidx_start = sb_vert_start_idx[sb_idx];
        auto& sb = soft_bodies[sb_idx];
        for (int t = 0; t < sb.tetrahedrons.size(); t++) {
            glm::ivec4 tet = sb.tetrahedrons[t];
            glm::rvec3 f_g = (1. / 4.) * sb.props.density * sb.W[t] * gravity;
            f_s_tot_ptr[vidx_start + tet[0]] += f_g;
            f_s_tot_ptr[vidx_start + tet[1]] += f_g;
            f_s_tot_ptr[vidx_start + tet[2]] += f_g;
            f_s_tot_ptr[vidx_start + tet[3]] += f_g;
        }
    }
    return f_s_tot;
}

void ArtWithSoftBodies::integrate_admm_coupled() {
    // Forward kinematics of articulation
    artsim::calc_transforms(art, x_r.data(), art_joint_trans.data(), art_link_trans.data());

    // Calculate coupling jacobian
    calc_constraint_jacobian();

    // Calculate articulation matrix M_r
    dynmat_view<real> M_r_view(M_r.data(), N_r, N_r);
    mass_matrix(art, dt, x_r.data(), M_r_view);

    // Calculate inverse of articulation matrix M_r^{-1}
    dynmat_view<real> M_r_inv_view(M_r_inv.data(), N_r, N_r);
    dynmat<real> identity(N_r, IDENTITY);
    multiply_inverse_mass_matrix(art, dt, x_r.data(), identity.to_view(), OUT M_r_inv_view);

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
    VectorXr v_r_dot(N_r);
    featherstone_forward_dynamics(art, gravity, dt, nullptr, x_r.data(), v_r.data(), f_r.data(), OUT v_r_dot.data());
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

        admm_local_solve((glm::rvec3*)x_s.data(), OUT z.data(), OUT u.data(), OUT F.data(), OUT F_svd.data());

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

    integrate_implicit_euler(art, dt, nullptr, x_r.data(), v_r.data());

    // Project constrained positions to articulation
    calc_transforms(art, x_r.data(), art_joint_trans.data(), nullptr);
    for (int cidx = 0; cidx < N_c; cidx++) {
        int vidx = index_c_to_s[cidx];
        int link_idx = index_c_to_link[cidx];
        auto T = art_joint_trans[link_idx] * constr_vertices_offset[cidx];
        x_s(3*vidx+0) = T.v[0];
        x_s(3*vidx+1) = T.v[1];
        x_s(3*vidx+2) = T.v[2];
    }
}

}
