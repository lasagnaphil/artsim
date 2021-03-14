//
// Created by lasagnaphil on 20. 10. 4..
//

#ifndef ARTSIM_ARTICULATION_STATE_H
#define ARTSIM_ARTICULATION_STATE_H

#include <artsim/artsim.h>
#include <artsim/dynamics.h>
#include <artsim/contacts.h>

#include <random>

namespace artsim {

struct ArticulationState {
    artsim::ArticulatedBody* art;
    artsim::MaterialDB* material_db;

    size_t num_pos_dofs;
    size_t num_vel_dofs;
    size_t num_joints;
    std::vector<real> q;
    std::vector<real> u;
    std::vector<real> udot;
    std::vector<real> tau;
    std::vector<glmx::tscrew<real>> f_ext;
    std::vector<glmx::ttransform<real>> T_link_global;
    std::vector<glmx::ttransform<real>> T_joint_global;

    glm::tvec3<real> gravity = {0, -9.81, 0};

    bool enable_collision_with_ground = false;
    std::vector<uint32_t> ground_col_enabled_links;
    std::vector<ContactPoint> contact_points;
    std::vector<tvec3<real>> contact_normals;

    ContactSolverType solver_type;
    uint32_t max_iters;

    ArticulationState() = default;

    ArticulationState(artsim::ArticulatedBody *artPtr, artsim::MaterialDB* material_db,
                      ContactSolverType solverType = ContactSolverType::NCP,
                      uint32_t max_iters = 4)
            : art(artPtr), material_db(material_db),
              num_pos_dofs(art->get_num_pos_dofs()), num_vel_dofs(art->get_num_vel_dofs()), num_joints(art->get_num_joints()),
              q(num_pos_dofs, 0), u(num_vel_dofs, 0), udot(num_vel_dofs, 0), tau(num_vel_dofs, 0),
              f_ext(num_joints, glmx::tscrew<real>(glmx::IDENTITY)),
              T_link_global(num_joints, glmx::ttransform<real>(glmx::IDENTITY)),
              T_joint_global(num_joints, glmx::ttransform<real>(glmx::IDENTITY)),
              solver_type(solverType), max_iters(max_iters)
    {
        reset_positions();
        for (uint32_t i = 0; i < num_joints; i++) {
            ground_col_enabled_links.push_back(i);
        }
    }

    void reset_positions() {
        real* qp = q.data();
        for (int i = 0; i < num_joints; i++) {
            switch (art->joints[i].type) {
                JOINT_DOF_1_CASE {
                    qp[0] = 0;
                } break;
                case JOINT_TYPE_FLOATING: {
                    qp[0] = 0; qp[1] = 0; qp[2] = 0;
                    qp[3] = 0; qp[4] = 0; qp[5] = 0; qp[6] = 1;
                } break;
                case JOINT_TYPE_SPHERICAL: {
                    qp[0] = 0; qp[1] = 0; qp[2] = 0; qp[3] = 1;
                } break;
            }
            qp += art->joint_pos_dofs[i];
        }

        calc_transforms(*art, q.data(), OUT T_link_global.data(), OUT T_joint_global.data());
    }

    void update_transforms() {
        calc_transforms(*art, q.data(), OUT T_link_global.data(), OUT T_joint_global.data());
    }

    void randomize_positions() {
        static std::default_random_engine engine(0);

        const real pi = glm::pi<real>();
        real* qp = q.data();
        for (int i = 0; i < num_joints; i++) {
            switch (art->joints[i].type) {
                JOINT_DOF_1_CASE {
                    qp[0] = std::uniform_real_distribution<real>(-0.2*pi, 0.2*pi)(engine);
                } break;
                case JOINT_TYPE_SPHERICAL: {
                    real len = std::uniform_real_distribution<real>(-0.2*pi, 0.2*pi)(engine);
                    glm::tvec3<real> dir = glm::tvec3<real>(
                            std::uniform_real_distribution<real>(-1, 1)(engine),
                            std::uniform_real_distribution<real>(-1, 1)(engine),
                            std::uniform_real_distribution<real>(-1, 1)(engine)
                    );
                    glm::tquat<real> vexp = artsim::exp(len * normalize(dir));
                    qp[0] = vexp[0]; qp[1] = vexp[1]; qp[2] = vexp[2]; qp[3] = vexp[3];
                } break;
                case JOINT_TYPE_FLOATING: {
                    qp[0] = std::uniform_real_distribution<real>(-0.1, 0.1)(engine);
                    qp[1] = std::uniform_real_distribution<real>(num_joints, num_joints+1)(engine);
                    qp[2] = std::uniform_real_distribution<real>(-0.1, 0.1)(engine);

                    real len = std::uniform_real_distribution<real>(-0.2f*pi, 0.2f*pi)(engine);
                    glm::tvec3<real> dir = glm::tvec3<real>(
                            std::uniform_real_distribution<real>(-1, 1)(engine),
                            std::uniform_real_distribution<real>(-1, 1)(engine),
                            std::uniform_real_distribution<real>(-1, 1)(engine)
                    );
                    glm::tquat<real> vexp = artsim::exp(len * normalize(dir));
                    qp[3] = vexp[0]; qp[4] = vexp[1]; qp[5] = vexp[2]; qp[6] = vexp[3];
                } break;
            }
            qp += art->joint_pos_dofs[i];
        }

        calc_transforms(*art, q.data(), OUT T_link_global.data(), OUT T_joint_global.data());
    }

    void simulate(real dt) {
        if (enable_collision_with_ground) {
#if 0
            contact_points = artsim::contact_points_between_art_links_and_ground(
                    *art, Id<ArticulatedBody>::null(),
                    ground_col_enabled_links.data(), ground_col_enabled_links.size(), T_link_global.data());
#else
            calc_transforms(*art, q.data(), OUT T_link_global.data(), OUT T_joint_global.data());
            for (int i = 0; i < num_joints; i++) {
                auto& link = art->links[i];
                link.bt_collision_object->setWorldTransform(btconv(T_link_global[i]));
            }
            auto bt_world = art->bt_collision_world;
            bt_world->performDiscreteCollisionDetection();

            contact_points = artsim::get_contact_points_bullet(bt_world);
#endif
            contact_normals.resize(contact_points.size());
        }

        artsim::euler_step_with_collision(solver_type, max_iters,
                                          *art, *material_db, gravity, dt, f_ext.data(), tau.data(),
                                          contact_points.data(), contact_points.size(),
                                          INOUT q.data(), INOUT u.data(),
                                          OUT udot.data(), OUT contact_normals.data());

        calc_transforms(*art, q.data(), OUT T_link_global.data(), OUT T_joint_global.data());
    }

    void simulate(real dt, int N) {
        for (int i = 0; i < N; i++) {
            simulate(dt);
        }
    }

    real get_joint_pos_1dof(int joint_idx) const {
        if (art->joint_pos_dofs[joint_idx] != 1) { return 0; }
        uint32_t jidx_start = art->joint_pos_dof_starts[joint_idx];
        return q[jidx_start];
    }

    glm::tquat<real> get_joint_pos_spherical(int joint_idx) const {
        if (art->joint_pos_dofs[joint_idx] != 1) { return glm::quat(1, 0, 0, 0); }
        uint32_t jidx_start = art->joint_pos_dof_starts[joint_idx];
        return glm::make_quat(q.data() + jidx_start);
    }

    glmx::ttransform<real> get_root_transform() const {
        if (!art->floating) return {};
        return {glm::make_vec3(q.data()), glm::mat3_cast(glm::make_quat(q.data() + 3))};
    }

    void set_joint_pos_1dof(int joint_idx, real qj) {
        if (art->joint_pos_dofs[joint_idx] != 1) { return; }
        uint32_t jidx_start = art->joint_pos_dof_starts[joint_idx];
        q[jidx_start] = qj;
    }

    void set_joint_pos_spherical(int joint_idx, glm::tquat<real> qj) {
        if (art->joint_pos_dofs[joint_idx] != 4) { return; }
        uint32_t jidx_start = art->joint_pos_dof_starts[joint_idx];
        q[jidx_start+0] = qj[0];
        q[jidx_start+1] = qj[1];
        q[jidx_start+2] = qj[2];
        q[jidx_start+3] = qj[3];
    }

    void set_root_transform(const glmx::ttransform<real>& rootT) {
        if (art->floating) {
            glm::quat rot = glm::quat_cast(rootT.R);
            q[0] = rootT.v[0];
            q[1] = rootT.v[1];
            q[2] = rootT.v[2];
            q[3] = rot[0];
            q[4] = rot[1];
            q[5] = rot[2];
            q[6] = rot[3];
        }
    }
};

}

#endif //ARTSIM_ARTICULATION_STATE_H

