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

    std::vector<int> constr_vertices;
    std::vector<std::pair<int, int>> constr_vertices_range;
    std::vector<int> sb_start_idx;

    int Ns, Nc, Nr;

    MatrixXr x_s, x_r, v_s, v_r, f_s, f_r;

    MatrixXr J_cr;
    MatrixXr M_r;
    MatrixXr M_r_inv;
    MatrixXr M_r_inv_J_cr_T;

public:
    void load(const char* metadata);

    int get_constr_vertices_of_soft_body_dof(int sb_idx) {
        auto range = constr_vertices_range[sb_idx];
        return range.second - range.first;
    }

    void get_constr_vertices_of_soft_body(int sb_idx, OUT int* vertices) {
        auto range = constr_vertices_range[sb_idx];
        for (int i = 0; i < range.second - range.first; i++) {
            vertices[i] = constr_vertices[range.first + i];
        }
    }

    int get_soft_body_dof(int sb_idx) {
        return sb_start_idx[sb_idx+1] - sb_start_idx[sb_idx];
    }
    real* get_soft_body_pos_buf(int sb_idx) {
        return x_s.data() + sb_start_idx[sb_idx];
    }
    real* get_soft_body_vel_buf(int sb_idx) {
        return v_s.data() + sb_start_idx[sb_idx];
    }

    int get_art_pos_dof() {
        return art.get_num_pos_dofs();
    }
    int get_art_vel_dof() {
        return art.get_num_vel_dofs();
    }
    real* get_art_pos() {
        return x_r.data();
    }
    real* get_art_vel() {
        return v_r.data();
    }
};

}

#endif //ARTSIM_ART_WITH_SOFT_BODIES_H
