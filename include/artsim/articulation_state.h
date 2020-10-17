//
// Created by lasagnaphil on 20. 10. 4..
//

#ifndef ARTSIM_ARTICULATION_STATE_H
#define ARTSIM_ARTICULATION_STATE_H

#include <artsim/artsim.h>
#include <artsim/dynamics.h>

#include <random>

namespace artsim {

template <class T>
struct ArticulationState {
    artsim::ArticulatedBody* art;

    size_t num_pos_dofs;
    size_t num_vel_dofs;
    size_t num_joints;
    std::vector<T> q;
    std::vector<T> u;
    std::vector<T> udot;
    std::vector<T> tau;
    std::vector<artsim::tscrew<T>> f_ext;
    std::vector<artsim::ttransform<T>> T_local;
    std::vector<artsim::ttransform<T>> T_global;

    glm::tvec3<T> gravity = {0, -9.81, 0};

    ArticulationState(artsim::ArticulatedBody *artPtr)
            : art(artPtr),
              num_pos_dofs(art->get_num_pos_dofs()), num_vel_dofs(art->get_num_vel_dofs()), num_joints(art->get_num_joints()),
              q(num_pos_dofs, 0), u(num_vel_dofs, 0), udot(num_vel_dofs, 0), tau(num_vel_dofs, 0),
              f_ext(num_joints, tscrew<T>()), T_local(num_joints, ttransform<T>()), T_global(num_joints, ttransform<T>())
    {
        reset_positions();
    }

    void reset_positions() {
        T* qp = q.data();
        for (int i = 0; i < num_joints; i++) {
            switch (art->joints[i].type) {
                case JointType::Prismatic: case JointType::Revolute: {
                    qp[0] = 0;
                } break;
                case JointType::Spherical: {
                    qp[0] = 0; qp[1] = 0; qp[2] = 0; qp[3] = 1;
                } break;
            }
            qp += art->joint_pos_dofs[i];
        }

        calc_transforms(*art, q.data(), T_local.data(), T_global.data());
    }

    void randomize_positions() {
        std::random_device r;
        std::default_random_engine engine(r());

        const T pi = glm::pi<T>();
        T* qp = q.data();
        for (int i = 0; i < num_joints; i++) {
            switch (art->joints[i].type) {
                case JointType::Prismatic: case JointType::Revolute: {
                    qp[0] = std::uniform_real_distribution<T>(-0.2f*pi, 0.2f*pi)(engine);
                } break;
                case JointType::Spherical: {
                    T len = std::uniform_real_distribution<T>(-0.2f*pi, 0.2f*pi)(engine);
                    glm::tvec3<T> dir = glm::tvec3<T>(
                            std::uniform_real_distribution<T>(-1, 1)(engine),
                            std::uniform_real_distribution<T>(-1, 1)(engine),
                            std::uniform_real_distribution<T>(-1, 1)(engine)
                    );
                    glm::tquat<T> vexp = artsim::exp(len * normalize(dir));
                    qp[0] = vexp[0]; qp[1] = vexp[1]; qp[2] = vexp[2]; qp[3] = vexp[3];
                } break;
            }
            qp += art->joint_pos_dofs[i];
        }

        calc_transforms(*art, q.data(), T_local.data(), T_global.data());
    }

    void simulate(T dt) {
        artsim::featherstone_forward_dynamics(*art, gravity, f_ext.data(), q.data(), u.data(), tau.data(), OUT udot.data());
        // artsim::forward_dynamics_using_rnea(*art, gravity, f_ext.data(), q.data(), qdot.data(), tau.data(), OUT q2dot.data());
        artsim::integrate_implicit_euler(*art, dt, udot.data(), OUT q.data(), OUT u.data());
        calc_transforms(*art, q.data(), T_local.data(), T_global.data());
    }

    void simulate(T dt, int N) {
        for (int i = 0; i < N; i++) {
            artsim::featherstone_forward_dynamics(*art, gravity, f_ext.data(), q.data(), u.data(), tau.data(), OUT udot.data());
            // artsim::forward_dynamics_using_rnea(*art, gravity, f_ext.data(), q.data(), qdot.data(), tau.data(), OUT q2dot.data());
            artsim::integrate_implicit_euler(*art, dt, udot.data(), OUT q.data(), OUT u.data());
        }
        calc_transforms(*art, q.data(), T_local.data(), T_global.data());
    }

    T get_joint_pos_1dof(int joint_idx) {
        if (art->joint_pos_dofs[joint_idx] != 1) { return 0; }
        uint32_t jidx_start = art->joint_pos_dof_starts[joint_idx];
        return q[jidx_start];
    }

    glm::tquat<T> get_joint_pos_spherical(int joint_idx) {
        if (art->joint_pos_dofs[joint_idx] != 1) { return glm::quat(1, 0, 0, 0); }
        uint32_t jidx_start = art->joint_pos_dof_starts[joint_idx];
        return glm::make_quat(q.data() + jidx_start);
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
};

}

#endif //ARTSIM_ARTICULATION_STATE_H

