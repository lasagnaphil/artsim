//
// Created by lasagnaphil on 21. 3. 12..
//

#ifndef ARTSIM_ART_WITH_SOFT_BODIES_H
#define ARTSIM_ART_WITH_SOFT_BODIES_H

#include <artsim/soft_body.h>
#include <tinyxml2.h>
#include <memory>

namespace artsim {

using MatrixXr = Eigen::Matrix<real, Eigen::Dynamic, Eigen::Dynamic>;
using VectorXr = Eigen::Matrix<real, Eigen::Dynamic, 1>;

struct ArtWithSoftBodies {
protected:
    std::unique_ptr<tinyxml2::XMLDocument> doc;

    ArticulatedBody art;
    std::vector<SoftBodyData> soft_bodies;
    std::vector<ADMMConstraints> sb_constraints;
    std::vector<std::string> sb_names;

    std::vector<int> index_s_to_c;
    std::vector<int> index_c_to_link;
    std::vector<int> index_c_to_s;

    std::vector<int> sb_vert_start_idx;
    std::vector<int> sb_tet_start_idx;

    std::vector<std::map<int, std::vector<int>>> sb_constr_vertices;

    real dt;
    glm::rvec3 gravity;

    int N_s, N_f, N_c, N_r;
    int N_t;

    VectorXr x_s, x_r, v_s, v_r, f_s, f_r;
    std::vector<glmx::ttransform<real>> art_link_trans;
    std::vector<glmx::ttransform<real>> art_joint_trans;
    std::vector<glmx::tscrew<real>> art_joint_S;
    MatrixXr J_cr;
    MatrixXr M_r;
    MatrixXr M_r_inv;
    MatrixXr M_r_inv_J_cr_T;

    std::vector<glmx::ttransform<real>> constr_vertices_offset;

public:
    void load(const char* metadata);

    void save(const char* metadata);

    void update_attachments();

    void reset();

    void integrate_admm_coupled();

    const std::vector<SoftBodyData>& get_soft_bodies() {
        return soft_bodies;
    }
    int get_num_soft_bodies() {
        return soft_bodies.size();
    }

    int get_soft_body_dof(int sb_idx) {
        return sb_vert_start_idx[sb_idx + 1] - sb_vert_start_idx[sb_idx];
    }
    int get_soft_body_start_vidx(int sb_idx) {
        return sb_vert_start_idx[sb_idx];
    }

    std::map<int, std::vector<int>> get_soft_body_attachments(int sb_idx) {
        return sb_constr_vertices[sb_idx];
    }

    int get_total_soft_body_dof() { return N_s; }
    int get_total_free_soft_body_dof() { return N_f; }
    int get_total_constrained_soft_body_dof() { return N_c; }

    glm::rvec3* get_soft_body_pos_buf(int sb_idx) {
        return reinterpret_cast<glm::rvec3*>(x_s.data()) + sb_vert_start_idx[sb_idx];
    }
    glm::rvec3* get_soft_body_vel_buf(int sb_idx) {
        return reinterpret_cast<glm::rvec3*>(v_s.data()) + sb_vert_start_idx[sb_idx];
    }
    glm::rvec3* get_soft_body_force_buf(int sb_idx) {
        return reinterpret_cast<glm::rvec3*>(f_s.data()) + sb_vert_start_idx[sb_idx];
    }
    const int* get_soft_body_s_to_c_buf() const {
        return index_s_to_c.data();
    }
    const int* get_soft_body_c_to_s_buf() const {
        return index_c_to_s.data();
    }
    const char* get_soft_body_name(int sb_idx) {
        return sb_names[sb_idx].c_str();
    }

    const ArticulatedBody& get_articulation() {
        return art;
    }
    int get_art_pos_dof() {
        return art.get_num_pos_dofs();
    }
    int get_art_vel_dof() {
        return art.get_num_vel_dofs();
    }
    real* get_art_pos_buf() {
        return x_r.data();
    }
    real* get_art_vel_buf() {
        return v_r.data();
    }
    real* get_art_force_buf() {
        return f_r.data();
    }
    glmx::rtransform get_link_trans(int link_idx) {
        assert(link_idx >= 0 && link_idx < N_r);
        return art_link_trans[link_idx];
    }
    glmx::rtransform get_joint_trans(int joint_idx) {
        assert(joint_idx >= 0 && joint_idx < N_r);
        return art_joint_trans[joint_idx];
    }

    real get_sim_deltatime() const { return dt; }
    void set_sim_deltatime(real dt) { this->dt = dt; }

    glm::rvec3 get_gravity() const { return gravity; }
    void set_gravity(glm::rvec3 g) { gravity = g; }


protected:
    void admm_local_solve(const glm::tvec3<real>* x,
            OUT glm::tmat3x3<real>* z, OUT glm::tmat3x3<real>* u,
            OUT glm::tmat3x3<real>* F, OUT glmx::SVD_mats<real>* F_svd);

    void admm_update_b(real dt, const glm::tmat3x3<real>* z, const glm::tmat3x3<real>* u, const glm::tvec3<real>* x0,
            INOUT real* b);

    void admm_update_residuals(const glm::tmat3x3<real>* z_prev, const glm::tmat3x3<real>* z_next, const glm::tvec3<real>* x,
            INOUT real& primal_res_sq, INOUT real& dual_res_sq);

    void apply_selector_matrix(const real* X_s, OUT real* X_c);
    void apply_selector_matrix_inv(const real* X_c, OUT real* X_s);
    void apply_selector_matrix_add(INOUT real* X_s, const real* dX_c);
    void apply_selector_matrix_sub(INOUT real* X_s, const real* dX_c);
    void calc_constraint_jacobian();
    VectorXr calc_total_force_with_gravity();
};

}

#endif //ARTSIM_ART_WITH_SOFT_BODIES_H
