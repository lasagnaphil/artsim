//
// Created by Phillip Chang on 2020/09/27.
//

#include "artsim/example_articulations.h"

artsim::ArticulationState::ArticulationState(artsim::ArticulatedBody *artPtr)
        : art(artPtr), num_dofs(art->get_num_dofs()), num_joints(art->get_num_joints()),
          q(num_dofs, 0), qdot(num_dofs, 0), q2dot(num_dofs, 0), tau(num_dofs, 0),
          f_ext(num_joints, screw()), T_local(num_joints, transform()), T_global(num_joints, transform())
{
    calc_transforms(*art, q.data(), T_local.data(), T_global.data());
}

void artsim::ArticulationState::simulate(float dt) {
    // artsim::featherstone_forward_dynamics(*art, gravity, f_ext.data(), q.data(), qdot.data(), tau.data(), OUT q2dot.data());
    artsim::forward_dynamics_using_rnea(*art, gravity, f_ext.data(), q.data(), qdot.data(), tau.data(), OUT q2dot.data());
    artsim::integrate_implicit_euler(*art, dt, q2dot.data(), OUT q.data(), OUT qdot.data());
    calc_transforms(*art, q.data(), T_local.data(), T_global.data());
}

void artsim::ArticulationState::simulate(float dt, int N) {
    for (int i = 0; i < N; i++) {
        // artsim::featherstone_forward_dynamics(*art, gravity, f_ext.data(), q.data(), qdot.data(), tau.data(), OUT q2dot.data());
        artsim::forward_dynamics_using_rnea(*art, gravity, f_ext.data(), q.data(), qdot.data(), tau.data(), OUT q2dot.data());
        artsim::integrate_implicit_euler(*art, dt, q2dot.data(), OUT q.data(), OUT qdot.data());
    }
    calc_transforms(*art, q.data(), T_local.data(), T_global.data());
}

artsim::ArticulatedBody artsim::examples::create_double_pendulum_ball(float m1, float m2, float l1, float l2) {
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

artsim::ArticulatedBody artsim::examples::create_double_pendulum_link(float density, float l1, float l2, float d) {
    Shape box1 = Shape::make_box({d, l1, d});
    Shape box2 = Shape::make_box({d, l2, d});

    ArticulatedBody art;
    art.add_link_and_joint(
            Link::create(box1.inertia(density), box1.mass(density), box1,
                         transform(glm::vec3(0.0f, -l1/2, 0.0f)),
                         transform(glm::vec3(0.0f, l1/2, 0.0f)),
                         -1, {}),
            Joint::revolute_free(Ez<float>())
    );
    art.add_link_and_joint(
            Link::create(box2.inertia(density), box2.mass(density), box2,
                         transform(glm::vec3(0.0f, -(l1+l2)/2, 0.0f)),
                         transform(glm::vec3(0.0f, l2/2, 0.0f)),
                         0, {}),
            Joint::revolute_free(Ez<float>())
    );
    art.setup();
    return art;
}

artsim::ArticulatedBody
artsim::examples::create_triple_pendulum_link(float density, float l1, float l2, float l3, float d) {
    Shape box1 = Shape::make_box({d, l1, d});
    Shape box2 = Shape::make_box({d, l2, d});
    Shape box3 = Shape::make_box({d, l3, d});

    ArticulatedBody art;
    art.add_link_and_joint(
            Link::create(box1.inertia(density), box1.mass(density), box1,
                         transform(glm::vec3(0.0f, -l1/2, 0.0f)),
                         transform(glm::vec3(0.0f, l1/2, 0.0f)),
                         -1, {}),
            Joint::revolute_free(Ez<float>())
    );
    art.add_link_and_joint(
            Link::create(box2.inertia(density), box2.mass(density), box2,
                         transform(glm::vec3(0.0f, -(l1+l2)/2, 0.0f)),
                         transform(glm::vec3(0.0f, l2/2, 0.0f)),
                         0, {}),
            Joint::revolute_free(Ez<float>())
    );
    art.add_link_and_joint(
            Link::create(box3.inertia(density), box3.mass(density), box2,
                         transform(glm::vec3(0.0f, -(l2+l3)/2, 0.0f)),
                         transform(glm::vec3(0.0f, l3/2, 0.0f)),
                         1, {}),
            Joint::revolute_free(Ez<float>())
    );
    art.setup();
    return art;
}

artsim::ArticulatedBody artsim::examples::create_furuta_pendulum(float density, float l1, float l2, float d) {
    Shape box1 = Shape::make_box({l1, d, d});
    Shape box2 = Shape::make_box({d, l2, d});

    ArticulatedBody art;
    art.add_link_and_joint(
            Link::create(box1.inertia(density), box1.mass(density), box1,
                         transform(glm::vec3(l1/2, 0.0f, 0.0f)),
                         transform(glm::vec3(-l1/2, 0.0f, 0.0f)),
                         -1, {}),
            Joint::revolute_free(Ey<float>())
    );
    art.add_link_and_joint(
            Link::create(box2.inertia(density), box2.mass(density), box2,
                         transform(glm::vec3(l1/2, -l2/2, 0.0f)),
                         transform(glm::vec3(0.f, l2/2, 0.0f)),
                         0, {}),
            Joint::revolute_free(Ex<float>())
    );
    art.setup();
    return art;
}

artsim::ArticulatedBody artsim::examples::create_5_link_tree_revolute(float density) {
    Shape box = Shape::make_box({0.1f, 1.0f, 0.1f});

    ArticulatedBody art;
    art.add_link_and_joint(
            Link::create(box.inertia(density), box.mass(density), box,
                         transform(glm::vec3(0.0f, -0.5f, 0.0f)),
                         transform(glm::vec3(0.0f, 0.5f, 0.0f)),
                         -1, {}),
            Joint::revolute_free(Ez<float>())
    );
    art.add_link_and_joint(
            Link::create(box.inertia(density), box.mass(density), box,
                         transform(glm::vec3(0.0f, -1.0f, 0.0f)),
                         transform(glm::vec3(0.0f, 0.5f, 0.0f)),
                         0, {}),
            Joint::revolute_free(Ez<float>())
    );
    art.add_link_and_joint(
            Link::create(box.inertia(density), box.mass(density), box,
                         transform(glm::vec3(0.0f, -1.0f, 0.0f)),
                         transform(glm::vec3(0.0f, 0.5f, 0.0f)),
                         1, {}),
            Joint::revolute_free(Ez<float>())
    );
    art.add_link_and_joint(
            Link::create(box.inertia(density), box.mass(density), box,
                         transform(glm::vec3(0.0f, -1.0f, 0.0f)),
                         transform(glm::vec3(0.0f, 0.5f, 0.0f)),
                         0, {}),
            Joint::revolute_free(Ex<float>())
    );
    art.add_link_and_joint(
            Link::create(box.inertia(density), box.mass(density), box,
                         transform(glm::vec3(0.0f, -1.0f, 0.0f)),
                         transform(glm::vec3(0.0f, 0.5f, 0.0f)),
                         3, {}),
            Joint::revolute_free(Ex<float>())
    );
    art.setup();
    return art;
}

artsim::ArticulatedBody artsim::examples::create_13_link_tree_revolute(float density) {
    Shape box = Shape::make_box({0.1f, 1.0f, 0.1f});

    ArticulatedBody art;

    art.add_link_and_joint(
            Link::create(box.inertia(density), box.mass(density), box,
                         transform(glm::vec3(0.0f, -0.5f, 0.0f)),
                         transform(glm::vec3(0.0f, 0.5f, 0.0f)),
                         -1, {}),
            Joint::revolute_free(Ez<float>())
    );

    auto add_link = [&](int parent, Joint joint) {
        art.add_link_and_joint(
                Link::create(box.inertia(density), box.mass(density), box,
                             transform(glm::vec3(0.0f, -1.0f, 0.0f)),
                             transform(glm::vec3(0.0f, 0.5f, 0.0f)),
                             parent, {}),
                joint
        );
    };

    add_link(0, Joint::revolute_free(Ex<float>())); // 1
    add_link(1, Joint::revolute_free(Ez<float>())); // 2
    add_link(0, Joint::revolute_free(Ex<float>())); // 3
    add_link(3, Joint::revolute_free(Ez<float>())); // 4
    add_link(2, Joint::revolute_free(Ex<float>())); // 5
    add_link(5, Joint::revolute_free(Ez<float>())); // 6
    add_link(2, Joint::revolute_free(Ex<float>())); // 7
    add_link(7, Joint::revolute_free(Ez<float>())); // 8
    add_link(4, Joint::revolute_free(Ex<float>())); // 9
    add_link(9, Joint::revolute_free(Ez<float>())); // 10
    add_link(4, Joint::revolute_free(Ex<float>())); // 11
    add_link(11, Joint::revolute_free(Ez<float>())); // 12

    art.setup();
    return art;
}
