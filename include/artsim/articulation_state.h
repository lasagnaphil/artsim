//
// Created by lasagnaphil on 20. 10. 4..
//

#ifndef ARTSIM_ARTICULATION_STATE_H
#define ARTSIM_ARTICULATION_STATE_H

#include <artsim/artsim.h>
#include <artsim/dynamics.h>
#include <artsim/collision.h>

#include <random>

namespace artsim {

template <class T>
struct ArticulationState {
    artsim::ArticulatedBody* art;
    artsim::MaterialDB* material_db;

    size_t num_pos_dofs;
    size_t num_vel_dofs;
    size_t num_joints;
    std::vector<T> q;
    std::vector<T> u;
    std::vector<T> udot;
    std::vector<T> tau;
    std::vector<artsim::tscrew<T>> f_ext;
    std::vector<artsim::ttransform<T>> T_link_global;
    std::vector<artsim::ttransform<T>> T_joint_global;

    glm::tvec3<T> gravity = {0, -9.81, 0};

    bool enable_collision_with_ground = false;
    std::vector<uint32_t> ground_col_enabled_links;
    std::vector<ContactPoint> contact_points;
    std::vector<tvec3<T>> contact_normals;

    ContactSolverType solverType;

    ArticulationState() = default;

    ArticulationState(artsim::ArticulatedBody *artPtr, artsim::MaterialDB* material_db,
                      ContactSolverType solverType = ContactSolverType::NCP)
            : art(artPtr), material_db(material_db),
              num_pos_dofs(art->get_num_pos_dofs()), num_vel_dofs(art->get_num_vel_dofs()), num_joints(art->get_num_joints()),
              q(num_pos_dofs, 0), u(num_vel_dofs, 0), udot(num_vel_dofs, 0), tau(num_vel_dofs, 0),
              f_ext(num_joints, tscrew<T>()),
              T_link_global(num_joints, ttransform<T>()),
              T_joint_global(num_joints, ttransform<T>()),
              solverType(solverType)
    {
        reset_positions();
        for (uint32_t i = 0; i < num_joints; i++) {
            ground_col_enabled_links.push_back(i);
        }
    }

    void reset_positions() {
        T* qp = q.data();
        for (int i = 0; i < num_joints; i++) {
            switch (art->joints[i].type) {
                case JointType::Floating: {
                    qp[0] = 0; qp[1] = 0; qp[2] = 0;
                    qp[3] = 0; qp[4] = 0; qp[5] = 0; qp[6] = 1;
                }
                case JointType::Prismatic: case JointType::Revolute: {
                    qp[0] = 0;
                } break;
                case JointType::Spherical: {
                    qp[0] = 0; qp[1] = 0; qp[2] = 0; qp[3] = 1;
                } break;
            }
            qp += art->joint_pos_dofs[i];
        }

        calc_transforms(*art, q.data(), OUT T_link_global.data(), OUT T_joint_global.data());
    }

    void randomize_positions() {
        static std::default_random_engine engine(0);

        const T pi = glm::pi<T>();
        T* qp = q.data();
        for (int i = 0; i < num_joints; i++) {
            switch (art->joints[i].type) {
                case JointType::Prismatic: case JointType::Revolute: {
                    qp[0] = std::uniform_real_distribution<T>(-0.2*pi, 0.2*pi)(engine);
                } break;
                case JointType::Spherical: {
                    T len = std::uniform_real_distribution<T>(-0.2*pi, 0.2*pi)(engine);
                    glm::tvec3<T> dir = glm::tvec3<T>(
                            std::uniform_real_distribution<T>(-1, 1)(engine),
                            std::uniform_real_distribution<T>(-1, 1)(engine),
                            std::uniform_real_distribution<T>(-1, 1)(engine)
                    );
                    glm::tquat<T> vexp = artsim::exp(len * normalize(dir));
                    qp[0] = vexp[0]; qp[1] = vexp[1]; qp[2] = vexp[2]; qp[3] = vexp[3];
                } break;
                case JointType::Floating: {
                    int num_joints = art->joints.size();
                    qp[0] = std::uniform_real_distribution<T>(-0.1, 0.1)(engine);
                    qp[1] = std::uniform_real_distribution<T>(num_joints, num_joints+1)(engine);
                    qp[2] = std::uniform_real_distribution<T>(-0.1, 0.1)(engine);

                    T len = std::uniform_real_distribution<T>(-0.2f*pi, 0.2f*pi)(engine);
                    glm::tvec3<T> dir = glm::tvec3<T>(
                            std::uniform_real_distribution<T>(-1, 1)(engine),
                            std::uniform_real_distribution<T>(-1, 1)(engine),
                            std::uniform_real_distribution<T>(-1, 1)(engine)
                    );
                    glm::tquat<T> vexp = artsim::exp(len * normalize(dir));
                    qp[3] = vexp[0]; qp[4] = vexp[1]; qp[5] = vexp[2]; qp[6] = vexp[3];
                }
            }
            qp += art->joint_pos_dofs[i];
        }

        calc_transforms(*art, q.data(), OUT T_link_global.data(), OUT T_joint_global.data());
    }

    void simulate(T dt) {
        if (enable_collision_with_ground) {
            contact_points = artsim::contact_points_between_art_links_and_ground(
                    *art, Id<ArticulatedBody>::null(),
                    ground_col_enabled_links.data(), ground_col_enabled_links.size(), T_link_global.data());
            contact_normals.resize(contact_points.size());
        }

        switch (solverType) {
            case ContactSolverType::PGS:
                artsim::euler_step_with_collision<T, ContactSolverType::PGS>(
                        *art, *material_db, gravity, dt, f_ext.data(), tau.data(),
                        contact_points.data(), contact_points.size(),
                        INOUT q.data(), INOUT u.data(),
                        OUT udot.data(), OUT contact_normals.data()); break;
            case ContactSolverType::Bisection:
                artsim::euler_step_with_collision<T, ContactSolverType::Bisection>(
                        *art, *material_db, gravity, dt, f_ext.data(), tau.data(),
                        contact_points.data(), contact_points.size(),
                        INOUT q.data(), INOUT u.data(),
                        OUT udot.data(), OUT contact_normals.data()); break;
            case ContactSolverType::NCP:
                artsim::euler_step_with_collision<T, ContactSolverType::NCP>(
                        *art, *material_db, gravity, dt, f_ext.data(), tau.data(),
                        contact_points.data(), contact_points.size(),
                        INOUT q.data(), INOUT u.data(),
                        OUT udot.data(), OUT contact_normals.data()); break;
        }

        calc_transforms(*art, q.data(), OUT T_link_global.data(), OUT T_joint_global.data());
    }

    void simulate(T dt, int N) {
        for (int i = 0; i < N; i++) {
            simulate(dt);
        }
    }

    T get_joint_pos_1dof(int joint_idx) const {
        if (art->joint_pos_dofs[joint_idx] != 1) { return 0; }
        uint32_t jidx_start = art->joint_pos_dof_starts[joint_idx];
        return q[jidx_start];
    }

    glm::tquat<T> get_joint_pos_spherical(int joint_idx) const {
        if (art->joint_pos_dofs[joint_idx] != 1) { return glm::quat(1, 0, 0, 0); }
        uint32_t jidx_start = art->joint_pos_dof_starts[joint_idx];
        return glm::make_quat(q.data() + jidx_start);
    }

    ttransform<T> get_root_transform() const {
        if (!art->floating) return {};
        return {glm::make_vec3(q.data()), glm::mat3_cast(glm::make_quat(q.data() + 3))};
    }

    void set_joint_pos_1dof(int joint_idx, T qj) {
        if (art->joint_pos_dofs[joint_idx] != 1) { return; }
        uint32_t jidx_start = art->joint_pos_dof_starts[joint_idx];
        q[jidx_start] = qj;
    }

    void set_joint_pos_spherical(int joint_idx, glm::tquat<T> qj) {
        if (art->joint_pos_dofs[joint_idx] != 4) { return; }
        uint32_t jidx_start = art->joint_pos_dof_starts[joint_idx];
        q[jidx_start+0] = qj[0];
        q[jidx_start+1] = qj[1];
        q[jidx_start+2] = qj[2];
        q[jidx_start+3] = qj[3];
    }

    void set_root_transform(const ttransform<T>& rootT) {
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

