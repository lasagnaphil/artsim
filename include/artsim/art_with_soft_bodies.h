//
// Created by lasagnaphil on 21. 3. 12..
//

#ifndef ARTSIM_ART_WITH_SOFT_BODIES_H
#define ARTSIM_ART_WITH_SOFT_BODIES_H

#include <artsim/soft_body.h>

namespace artsim {

using MatrixXr = Eigen::Matrix<real, Eigen::Dynamic, Eigen::Dynamic>;
using VectorXr = Eigen::Matrix<real, Eigen::Dynamic, 1>;

struct ArtWithSoftBodies {
private:
    ArticulatedBody art;
    std::vector<SoftBodyData> soft_bodies;
    std::vector<ADMMConstraints> sb_constraints;

    std::vector<glm::ivec4> tetrahedra;

    std::vector<int> index_s_to_c;
    std::vector<int> index_c_to_link;
    std::vector<int> index_c_to_s;

    std::vector<int> sb_vert_start_idx;
    std::vector<int> sb_tet_start_idx;

    real dt;
    glm::rvec3 gravity;

    int N_s, N_f, N_c, N_r;
    int N_t;

    VectorXr x_s, x_r, v_s, v_r, f_s, f_r;
    std::vector<glmx::ttransform<real>> art_joint_trans;
    std::vector<glmx::tscrew<real>> art_joint_S;
    MatrixXr J_cr;
    MatrixXr M_r;
    MatrixXr M_r_inv;
    MatrixXr M_r_inv_J_cr_T;

    std::vector<glmx::ttransform<real>> constr_vertices_offset;

public:
    void load(const char* metadata);

    void reset();

    void integrate();

    const std::vector<SoftBodyData>& get_soft_bodies() {
        return soft_bodies;
    }
    int get_num_soft_bodies() {
        return soft_bodies.size();
    }
    int get_soft_body_dof(int sb_idx) {
        return sb_vert_start_idx[sb_idx + 1] - sb_vert_start_idx[sb_idx];
    }
    glm::rvec3* get_soft_body_pos_buf(int sb_idx) {
        return reinterpret_cast<glm::rvec3*>(x_s.data()) + sb_vert_start_idx[sb_idx];
    }
    glm::rvec3* get_soft_body_vel_buf(int sb_idx) {
        return reinterpret_cast<glm::rvec3*>(v_s.data()) + sb_vert_start_idx[sb_idx];
    }
    glm::rvec3* get_soft_body_force_buf(int sb_idx) {
        return reinterpret_cast<glm::rvec3*>(f_s.data()) + sb_vert_start_idx[sb_idx];
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

    real get_sim_deltatime() const { return dt; }
    void set_sim_deltatime(real dt) { this->dt = dt; }

    glm::rvec3 get_gravity() const { return gravity; }
    void set_gravity(glm::rvec3 g) { gravity = g; }


private:
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
};

}

#endif //ARTSIM_ART_WITH_SOFT_BODIES_H
