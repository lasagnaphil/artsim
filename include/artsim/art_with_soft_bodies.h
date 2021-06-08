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
    std::shared_ptr<tinyxml2::XMLDocument> doc;

    ArticulatedBody art;
    std::vector<SoftBodyData> soft_bodies;
    std::vector<SoftBodyPrecalcData> soft_bodies_precalc;
    std::vector<ADMMConstraints> sb_constraints;
    std::vector<std::string> sb_names;

    std::vector<int> index_s_to_c;
    std::vector<int> index_c_to_link;
    std::vector<int> index_c_to_s;

    std::vector<int> sb_vert_start_idx;
    std::vector<int> sb_tet_start_idx;

    std::vector<std::map<int, std::vector<int>>> sb_constr_vertices;
    std::vector<std::vector<int>> index_link_to_sb;

    real dt;
    glm::rvec3 gravity;

    int N_s, N_f, N_c, N_r;
    int N_t;

    VectorXr x_s, v_s, f_s;
    MatrixXr J_cr;
    MatrixXr M_r;
    MatrixXr M_r_inv;
    MatrixXr M_r_inv_J_cr_T;

    std::vector<glmx::ttransform<real>> constr_vertices_offset;

public:
    virtual void load(const char* metadata);

    virtual void save(const char* metadata);

    void update_attachments();

    void reset();

    void integrate_admm_coupled();

    const std::vector<SoftBodyData>& get_soft_bodies() {
        return soft_bodies;
    }
    int get_num_soft_bodies() {
        return soft_bodies.size();
    }

    int get_soft_body_vert_dof(int sb_idx) {
        return sb_vert_start_idx[sb_idx + 1] - sb_vert_start_idx[sb_idx];
    }
    int get_soft_body_tet_dof(int sb_idx) {
        return sb_tet_start_idx[sb_idx + 1] - sb_tet_start_idx[sb_idx];
    }
    int get_soft_body_start_vidx(int sb_idx) {
        return sb_vert_start_idx[sb_idx];
    }
    int get_soft_body_start_tidx(int sb_idx) {
        return sb_tet_start_idx[sb_idx];
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

    int convert_s_to_c(int vidx) const { return index_s_to_c[vidx]; }
    int convert_c_to_s(int cidx) const { return index_c_to_s[cidx]; }


    const char* get_soft_body_name(int sb_idx) {
        return sb_names[sb_idx].c_str();
    }

    const ArticulatedBody& get_articulation() const {
        return art;
    }
    ArticulatedBody& get_articulation() {
        return art;
    }
    int get_art_pos_dof() {
        return art.get_num_pos_dofs();
    }
    int get_art_vel_dof() {
        return art.get_num_vel_dofs();
    }
    real* get_art_pos_buf() {
        return art.get_pos_buf();
    }
    real* get_art_vel_buf() {
        return art.get_vel_buf();
    }
    real* get_art_force_buf() {
        return art.get_internal_force_buf();
    }
    glmx::rtransform get_link_trans(int link_idx) {
        assert(link_idx >= 0 && link_idx < N_r);
        return art.get_global_link_trans(link_idx);
    }
    glmx::rtransform get_joint_trans(int joint_idx) {
        assert(joint_idx >= 0 && joint_idx < N_r);
        return art.get_global_joint_trans(joint_idx);
    }

    real get_sim_deltatime() const { return dt; }
    void set_sim_deltatime(real dt) { this->dt = dt; }

    glm::rvec3 get_gravity() const { return gravity; }
    void set_gravity(glm::rvec3 g) { gravity = g; }


protected:
    void admm_calc_deformation_field_and_svd(const glm::tmat3x3<real>* u, OUT glm::tmat3x3<real>* F, OUT glmx::SVD_mats<real>* F_svd);
    void admm_local_solve(
            const glm::tmat3x3<real>* F, const glmx::SVD_mats<real>* F_svd,
            OUT glm::tmat3x3<real>* z, OUT glm::tmat3x3<real>* u);

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
