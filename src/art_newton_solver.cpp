//
// Created by lasagnaphil on 21. 7. 16..
//

#include "artsim/art_newton_solver.h"
#include "artsim/art_dynamics.h"

namespace artsim {

using MatrixXr = Eigen::Matrix<real, Eigen::Dynamic, Eigen::Dynamic>;
using VectorXr = Eigen::Matrix<real, Eigen::Dynamic, 1>;

struct StateDOFMetadata {
private:
    std::unordered_map<BodyId, std::pair<int, int>> data;
    int total_dof;
public:
    void build(Arena<ArticulatedBody>& arts, Arena<RigidBody>& rbs) {
        int cur_dof = 0;
        arts.foreach_id_val([&](Id<ArticulatedBody> art_id, ArticulatedBody& art) {
            int dof = art.get_num_vel_dofs();
            auto body_id = BodyId::from_articulated_body(art_id);
            this->data.insert({body_id, std::make_pair(cur_dof, dof)});
            cur_dof += dof;
        });
        rbs.foreach_id_val([&](Id<RigidBody> rb_id, RigidBody& rb) {
            int dof = 6;
            auto body_id = BodyId::from_rigid_body(rb_id);
            this->data.insert({body_id, std::make_pair(cur_dof, dof)});
            cur_dof += dof;
        });
        this->total_dof = cur_dof;
    }

    std::pair<int, int> get_dof_starts_and_size(BodyId body_id) const {
        return data.at(body_id);
    }

    int get_total_dof() const {
        return total_dof;
    }
};

void World::newton_solver(const ContactPoint* contact_points, int num_contact_points) {
    StateDOFMetadata state_meta;
    state_meta.build(articulated_bodies, rigid_bodies);

    int total_dof = state_meta.get_total_dof();

    MatrixXr J_c(3*num_contact_points, total_dof);
    std::vector<glm::rvec3> contact_compl; // compliance

    for (int cidx = 0; cidx < num_contact_points; cidx++) {
        auto& cp = contact_points[cidx];
        if (cp.body1_id.is_articulation()) {
            auto [art_id, lidx] = cp.body1_id.get_articulation_id();
            auto art = get_articulated_body(art_id);
            auto& art_spec = art->get_spec();
            int num_art_vel_dofs = art->get_num_vel_dofs();
            std::vector<glm::rvec3> J(num_art_vel_dofs);
            glmx::dynmat_view<real> J_view((real*)J.data(), 3, num_art_vel_dofs);
            calc_linear_jacobian(art_spec, lidx,
                                 glmx::rtransform(cp.pos) / art->get_global_joint_trans(lidx),
                                 art->get_global_joint_trans_buf(), J_view);
            auto [dof_start, dof_size] = state_meta.get_dof_starts_and_size(cp.body1_id.get_body_id());
            for (int k = 0; k < num_art_vel_dofs; k++) {
                J_c(3*cidx+0, k) += 0;
                J_c(3*cidx+1, k) += 0;
                J_c(3*cidx+2, k) += glm::dot(J[k], cp.normal);
            }
        }
    }

}

}
