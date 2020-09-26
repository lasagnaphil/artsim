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

    size_t num_dofs;
    size_t num_joints;
    std::vector<float> q;
    std::vector<float> qdot;
    std::vector<float> q2dot;
    std::vector<float> tau;
    std::vector<artsim::tscrew<float>> f_ext;
    std::vector<artsim::ttransform<float>> T_local;
    std::vector<artsim::ttransform<float>> T_global;

    glm::vec3 gravity = {0.f, -9.81f, 0.f};

    ArticulationState(artsim::ArticulatedBody* artPtr)
      : art(artPtr), num_dofs(art->get_num_dofs()), num_joints(art->get_num_joints()),
        q(num_dofs, 0), qdot(num_dofs, 0), q2dot(num_dofs, 0), tau(num_dofs, 0),
        f_ext(num_joints, screw()), T_local(num_joints, transform()), T_global(num_joints, transform())
    {
    }

    void simulate(float dt) {
        artsim::featherstone_forward_dynamics(*art, gravity, f_ext.data(), q.data(), qdot.data(), tau.data(), OUT q2dot.data());
        // artsim::forward_dynamics_using_rnea(*art, gravity, f_ext.data(), q.data(), qdot.data(), tau.data(), OUT q2dot.data());
        artsim::integrate_implicit_euler(*art, dt, q2dot.data(), OUT q.data(), OUT qdot.data());
        calc_transforms(*art, q.data(), T_local.data(), T_global.data());
    }

    void simulate(float dt, int N) {
        for (int i = 0; i < N; i++) {
            artsim::featherstone_forward_dynamics(*art, gravity, f_ext.data(), q.data(), qdot.data(), tau.data(), OUT q2dot.data());
            // artsim::forward_dynamics_using_rnea(*art, gravity, f_ext.data(), q.data(), qdot.data(), tau.data(), OUT q2dot.data());
            artsim::integrate_implicit_euler(*art, dt, q2dot.data(), OUT q.data(), OUT qdot.data());
        }
        calc_transforms(*art, q.data(), T_local.data(), T_global.data());
    }
};

namespace examples {

ArticulatedBody create_double_pendulum_ball(float m1 = 1.0f, float m2 = 1.0f, float l1 = 1.0f, float l2 = 1.0f) {
    ArticulatedBody art;
    art.add_link_and_joint(
            Link::create(glm::mat3(0), m1, Shape::make_sphere(0.1f),
                         transform(glm::vec3(0.0f, -l1, 0.0f)),
                         transform(glm::vec3(0.0f, l1, 0.0f)),
                         -1, Id<artsim::Material>::null()),
            Joint::revolute_free(Ez<float>())
    );
    art.add_link_and_joint(
            Link::create(glm::mat3(0), m2, Shape::make_sphere(0.1f),
                         transform(glm::vec3(0.0f, -l2, 0.0f)),
                         transform(glm::vec3(0.0f, l2, 0.0f)),
                         0, Id<artsim::Material>::null()),
            Joint::revolute_free(Ez<float>())
    );
    art.setup();
    return art;
}

ArticulatedBody create_double_pendulum_link(float density = 1000.0f,
                                            float l1 = 1.0f, float l2 = 1.0f, float d = 0.1f) {
    Shape box1 = Shape::make_box({d, l1, d});
    Shape box2 = Shape::make_box({d, l2, d});

    ArticulatedBody art;
    art.add_link_and_joint(
            Link::create(glm::mat3(0), box1.mass(density), box1,
                                transform(glm::vec3(0.0f, -l1, 0.0f)),
                                transform(glm::vec3(0.0f, l1, 0.0f)),
                                -1, {}),
            Joint::revolute_free(Ez<float>())
    );
    art.add_link_and_joint(
            Link::create(glm::mat3(0), box2.mass(density), box2,
                                transform(glm::vec3(0.0f, -l2, 0.0f)),
                                transform(glm::vec3(0.0f, l2, 0.0f)),
                                0, {}),
            Joint::revolute_free(Ez<float>())
    );
    art.setup();
    return art;
}

ArticulatedBody create_triple_pendulum_link(float density = 1000.0f,
                                            float l1 = 1.0f, float l2 = 1.0f, float l3 = 1.0f, float d = 0.1f) {
    Shape box1 = Shape::make_box({d, l1, d});
    Shape box2 = Shape::make_box({d, l2, d});
    Shape box3 = Shape::make_box({d, l3, d});

    ArticulatedBody art;
    art.add_link_and_joint(
            Link::create(glm::mat3(0), box1.mass(density), box1,
                         transform(glm::vec3(0.0f, -l1, 0.0f)),
                         transform(glm::vec3(0.0f, l1, 0.0f)),
                         -1, {}),
            Joint::revolute_free(Ez<float>())
    );
    art.add_link_and_joint(
            Link::create(glm::mat3(0), box2.mass(density), box2,
                         transform(glm::vec3(0.0f, -l2, 0.0f)),
                         transform(glm::vec3(0.0f, l2, 0.0f)),
                         0, {}),
            Joint::revolute_free(Ez<float>())
    );
    art.add_link_and_joint(
            Link::create(glm::mat3(0), box3.mass(density), box2,
                         transform(glm::vec3(0.0f, -l3, 0.0f)),
                         transform(glm::vec3(0.0f, l3, 0.0f)),
                         1, {}),
            Joint::revolute_free(Ez<float>())
    );
    art.setup();
    return art;
}

/*
ArticulatedBody create_furuta_pendulum(float density = 1000.0f, float l1 = 1.0f, float l2 = 1.0f, float d = 0.1f)  {

}
 */

}

}

#endif //ARTSIM_EXAMPLE_ARTICULATIONS_H
