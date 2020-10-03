//
// Created by Phillip Chang on 2020/09/26.
//

#ifndef ARTSIM_EXAMPLE_ARTICULATIONS_H
#define ARTSIM_EXAMPLE_ARTICULATIONS_H

#include <vector>
#include <artsim/artsim.h>
#include <artsim/dynamics.h>

namespace artsim {

struct ArticulationState {
    artsim::ArticulatedBody* art;

    size_t num_pos_dofs;
    size_t num_vel_dofs;
    size_t num_joints;
    std::vector<float> q;
    std::vector<float> qdot;
    std::vector<float> q2dot;
    std::vector<float> tau;
    std::vector<artsim::tscrew<float>> f_ext;
    std::vector<artsim::ttransform<float>> T_local;
    std::vector<artsim::ttransform<float>> T_global;

    glm::vec3 gravity = {0.f, -9.81f, 0.f};

    ArticulationState(artsim::ArticulatedBody* artPtr);

    void reset_positions();

    void randomize_positions();

    void simulate(float dt);

    void simulate(float dt, int N);

    float get_joint_pos_1dof(int joint_idx);

    glm::quat get_joint_pos_spherical(int joint_idx);

    void set_joint_pos_1dof(int joint_idx, float qj);

    void set_joint_pos_spherical(int joint_idx, glm::quat qj);
};

namespace examples {

    ArticulatedBody create_single_pendulum_link(bool spherical,
                                                float density = 1000.0f, float l = 1.0f, float d = 0.1f);
    ArticulatedBody create_double_pendulum_ball(bool spherical, float m1 = 1.0f, float m2 = 1.0f, float l1 = 1.0f, float l2 = 1.0f);
    ArticulatedBody create_double_pendulum_link(bool spherical,
                                                float density = 1000.0f, float l1 = 1.0f, float l2 = 1.0f, float d = 0.1f);
    ArticulatedBody create_triple_pendulum_link(bool spherical,
                                                float density = 1000.0f, float l1 = 1.0f, float l2 = 1.0f, float l3 = 1.0f, float d = 0.1f);
    ArticulatedBody create_furuta_pendulum(bool spherical, float density = 1000.0f, float l1 = 1.0f, float l2 = 1.0f, float d = 0.1f);
    ArticulatedBody create_5_link_tree(bool spherical = false);
    ArticulatedBody create_13_link_tree(bool spherical = false);
}

}

#endif //ARTSIM_EXAMPLE_ARTICULATIONS_H
