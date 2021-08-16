//
// Created by Phillip Chang on 2020/09/27.
//

#include "artsim/utils/example_articulations.h"

using namespace glmx;

artsim::ArticulatedBodySpec artsim::examples::create_single_pendulum_link(bool spherical, float density, float l, float d) {
    ArticulatedBodySpec art;
    CollisionShape box1 = CollisionShape::make_box({d, l, d});
    art.add_link_and_joint(
            Link::create(box1, density,
                         ttransform<real>(glm::tvec3<real>(0.0f, 0.0f, 0.0f)),
                         ttransform<real>(glm::tvec3<real>(0.0f, -l, 0.0f)),
                         -1, Id<artsim::Material>::null()),
            spherical? Joint::spherical() : Joint::revolute_z());
    art.build();
    return art;
}

artsim::ArticulatedBodySpec artsim::examples::create_double_pendulum_ball(bool spherical, float m1, float m2, float l1, float l2) {
    ArticulatedBodySpec art;
    art.add_link_and_joint(
            Link::create(CollisionShape::make_sphere(0.1f), m1, tsmat3x3<real>(0),
                         ttransform<real>(glm::tvec3<real>(0.0f, 0.0f, 0.0f)),
                         ttransform<real>(glm::tvec3<real>(0.0f, -l1, 0.0f)),
                         -1, Id<artsim::Material>::null()),
            spherical? Joint::spherical() : Joint::revolute_z()
    );
    art.add_link_and_joint(
            Link::create(CollisionShape::make_sphere(0.1f), m2, tsmat3x3<real>(0),
                         ttransform<real>(glm::tvec3<real>(0.0f, -l1, 0.0f)),
                         ttransform<real>(glm::tvec3<real>(0.0f, -l2, 0.0f)),
                         0, Id<artsim::Material>::null()),
            spherical? Joint::spherical() : Joint::revolute_z()
    );
    art.build();
    return art;
}

artsim::ArticulatedBodySpec artsim::examples::create_double_pendulum_link(bool spherical, float density, float l1, float l2, float d) {
    CollisionShape box1 = CollisionShape::make_box({d, l1, d});
    CollisionShape box2 = CollisionShape::make_box({d, l2, d});

    ArticulatedBodySpec art;
    art.add_link_and_joint(
            Link::create(box1, density,
                         ttransform<real>(glm::tvec3<real>(0.0f, 0.0f, 0.0f)),
                         ttransform<real>(glm::tvec3<real>(0.0f, -l1/2, 0.0f)),
                         -1, {}),
            spherical? Joint::spherical() : Joint::revolute_z()
    );
    art.add_link_and_joint(
            Link::create(box2, density,
                         ttransform<real>(glm::tvec3<real>(0.0f, -l1, 0.0f)),
                         ttransform<real>(glm::tvec3<real>(0.0f, -l2/2, 0.0f)),
                         0, {}),
            spherical? Joint::spherical() : Joint::revolute_z()
    );
    art.build();
    return art;
}

artsim::ArticulatedBodySpec
artsim::examples::create_triple_pendulum_link(bool spherical, float density, float l1, float l2, float l3, float d) {
    CollisionShape box1 = CollisionShape::make_box({d, l1, d});
    CollisionShape box2 = CollisionShape::make_box({d, l2, d});
    CollisionShape box3 = CollisionShape::make_box({d, l3, d});

    ArticulatedBodySpec art;
    art.add_link_and_joint(
            Link::create(box1, density,
                         ttransform<real>(glm::tvec3<real>(0.0f, 0.0f, 0.0f)),
                         ttransform<real>(glm::tvec3<real>(0.0f, -l1/2, 0.0f)),
                         -1, {}),
            spherical? Joint::spherical() : Joint::revolute_z()
    );
    art.add_link_and_joint(
            Link::create(box2, density,
                         ttransform<real>(glm::tvec3<real>(0.0f, -l1, 0.0f)),
                         ttransform<real>(glm::tvec3<real>(0.0f, -l2/2, 0.0f)),
                         0, {}),
            spherical? Joint::spherical() : Joint::revolute_z()
    );
    art.add_link_and_joint(
            Link::create(box3, density,
                         ttransform<real>(glm::tvec3<real>(0.0f, -l2, 0.0f)),
                         ttransform<real>(glm::tvec3<real>(0.0f, -l3/2, 0.0f)),
                         1, {}),
            spherical? Joint::spherical() : Joint::revolute_z()
    );
    art.build();
    return art;
}

artsim::ArticulatedBodySpec artsim::examples::create_furuta_pendulum(bool spherical, float density, float l1, float l2, float d) {
    CollisionShape box1 = CollisionShape::make_box({l1, d, d});
    CollisionShape box2 = CollisionShape::make_box({d, l2, d});

    ArticulatedBodySpec art;
    art.add_link_and_joint(
            Link::create(box1, density,
                         ttransform<real>(glm::tvec3<real>(0.0f, 0.0f, 0.0f)),
                         ttransform<real>(glm::tvec3<real>(l1/2, 0.0f, 0.0f)),
                         -1, {}),
            Joint::revolute_y()
    );
    art.add_link_and_joint(
            Link::create(box2, density,
                         ttransform<real>(glm::tvec3<real>(l1, 0.0f, 0.0f)),
                         ttransform<real>(glm::tvec3<real>(0.f, -l2/2, 0.0f)),
                         0, {}),
            spherical? Joint::spherical() : Joint::revolute_x()
    );
    art.build();
    return art;
}

artsim::ArticulatedBodySpec artsim::examples::create_5_link_tree(bool spherical) {
    float density = 1000.0f;
    CollisionShape box = CollisionShape::make_box({0.1f, 1.0f, 0.1f});

    ArticulatedBodySpec art;

    art.add_link_and_joint(
            Link::create(box, density,
                         ttransform<real>(glm::tvec3<real>(0.0f, 0.0f, 0.0f)),
                         ttransform<real>(glm::tvec3<real>(0.0f, -0.5f, 0.0f)),
                         -1, {}),
            spherical? Joint::spherical() : Joint::revolute_z()
    );

    auto add_link = [&](int parent, Joint joint) {
        art.add_link_and_joint(
                Link::create(box, density,
                             ttransform<real>(glm::tvec3<real>(0.0f, -1.0f, 0.0f)),
                             ttransform<real>(glm::tvec3<real>(0.0f, -0.5f, 0.0f)),
                             parent, {}), joint);
    };

    add_link(0, spherical? Joint::spherical() : Joint::revolute_z());
    add_link(1, spherical? Joint::spherical() : Joint::revolute_x());
    add_link(0, spherical? Joint::spherical() : Joint::revolute_z());
    add_link(3, spherical? Joint::spherical() : Joint::revolute_x());

    art.build();
    return art;
}

artsim::ArticulatedBodySpec artsim::examples::create_13_link_tree(bool spherical) {
    float density = 1000.0f;
    CollisionShape box = CollisionShape::make_box({0.1f, 1.0f, 0.1f});

    ArticulatedBodySpec art;

    art.add_link_and_joint(
            Link::create(box, density,
                         ttransform<real>(glm::tvec3<real>(0.0f, 0.0f, 0.0f)),
                         ttransform<real>(glm::tvec3<real>(0.0f, -0.5f, 0.0f)),
                         -1, {}),
            spherical? Joint::spherical() : Joint::revolute_z()
    );

    auto add_link = [&](int parent, Joint joint) {
        art.add_link_and_joint(
                Link::create(box, density,
                             ttransform<real>(glm::tvec3<real>(0.0f, -1.0f, 0.0f)),
                             ttransform<real>(glm::tvec3<real>(0.0f, -0.5f, 0.0f)),
                             parent, {}), joint);
    };

    add_link(0, spherical? Joint::spherical() : Joint::revolute_x()); // 1
    add_link(1, spherical? Joint::spherical() : Joint::revolute_z()); // 2
    add_link(0, spherical? Joint::spherical() : Joint::revolute_x()); // 3
    add_link(3, spherical? Joint::spherical() : Joint::revolute_z()); // 4
    add_link(2, spherical? Joint::spherical() : Joint::revolute_x()); // 5
    add_link(5, spherical? Joint::spherical() : Joint::revolute_z()); // 6
    add_link(2, spherical? Joint::spherical() : Joint::revolute_x()); // 7
    add_link(7, spherical? Joint::spherical() : Joint::revolute_z()); // 8
    add_link(4, spherical? Joint::spherical() : Joint::revolute_x()); // 9
    add_link(9, spherical? Joint::spherical() : Joint::revolute_z()); // 10
    add_link(4, spherical? Joint::spherical() : Joint::revolute_x()); // 11
    add_link(11,spherical? Joint::spherical() : Joint::revolute_z()); // 12

    art.build();
    return art;
}

artsim::ArticulatedBodySpec artsim::examples::create_free_link(int num_links, bool spherical) {
    float density = 1000.0f;
    CollisionShape box = CollisionShape::make_box({0.1f, 1.0f, 0.1f});

    ArticulatedBodySpec art;

    art.add_link_and_joint(
            Link::create(box, density,
                         ttransform<real>(glm::tvec3<real>(0.0f, 0.0f, 0.0f)),
                         ttransform<real>(glm::tvec3<real>(0.0f, -0.5f, 0.0f)),
                         -1, {}),
            Joint::floating());

    for (int i = 1; i < num_links; i++) {
        art.add_link_and_joint(
                Link::create(box, density,
                             ttransform<real>(glm::tvec3<real>(0.0f, -1.0f, 0.0f)),
                             ttransform<real>(glm::tvec3<real>(0.0f, -0.5f, 0.0f)),
                             i-1, {}), spherical? Joint::spherical() : Joint::revolute_z());
    }

    art.build();
    return art;
}

artsim::ArticulatedBodySpec artsim::examples::create_free_ball(float radius) {
    float density = 1000.0f;
    CollisionShape shape = CollisionShape::make_sphere(radius);

    ArticulatedBodySpec art;

    art.add_link_and_joint(
            Link::create(shape, density,
                         ttransform<real>(glm::tvec3<real>(0.0f, 0.0f, 0.0f)),
                         ttransform<real>(glm::tvec3<real>(0.0f, 0.0f, 0.0f)),
                         -1, {}),
                         Joint::floating());
    art.build();
    return art;
}

