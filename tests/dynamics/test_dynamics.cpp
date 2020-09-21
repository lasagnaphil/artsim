//
// Created by Phillip Chang on 2020/09/20.
//

#include "doctest.h"
#include "artsim/artsim.h"
#include "artsim/dynamics.h"
#include "artsim/math/common.h"
using namespace artsim;

#include <chrono>

TEST_CASE("Double pendulum") {
    float m1 = 1.0f;
    float m2 = 1.0f;
    float l1 = 1.0f;
    float l2 = 2.0f;
    ArticulatedBody art;
    art.add_link_and_joint(
            Link::create(glm::mat3(0), m1, Shape::make_sphere(0.1f),
                         transform(glm::vec3(0.0f, -l1, 0.0f)),
                         transform(glm::vec3(0.0f, l1, 0.0f)),
                         -1, Id<Material>::null()),
            Joint::revolute_free(Ez<float>())
    );
    art.add_link_and_joint(
            Link::create(glm::mat3(0), m2, Shape::make_sphere(0.1f),
                         transform(glm::vec3(0.0f, -l2, 0.0f)),
                         transform(glm::vec3(0.0f, l2, 0.0f)),
                         0, Id<Material>::null()),
            Joint::revolute_free(Ez<float>())
    );
    art.setup();

    int dof = art.get_num_dofs();
    std::vector<float> q(dof, 0.0f);
    std::vector<float> qdot(dof, 0.0f);
    std::vector<float> q2dot(dof, 0.0f);
    std::vector<float> q2dot_1(dof, 0.0f);
    std::vector<float> q2dot_2(dof, 0.0f);
    std::vector<float> tau(dof, 0.0f);
    std::vector<screw> f_ext(art.get_num_joints(), screw());

    float g = 9.81f;
    float dt = 1.0f / 240.0f;

    std::vector<float> M(dof*dof, 0.0f);
    std::vector<float> h(dof, 0.0f);

    q[0] = 0.25f * glm::pi<float>();
    q[1] = 0.25f * glm::pi<float>();
    qdot[0] = 0.0f;
    qdot[1] = 0.0f;

    auto check_dp_M = [m1, m2, l1, l2](float* M, float theta1, float theta2) {
        REQUIRE(M[0] == doctest::Approx((m1+m2)*l1*l1 + m2*l2*l2 + 2*m2*l1*l2*cos(theta2)).epsilon(1e-6));
        REQUIRE(M[1] == doctest::Approx(m2*l2*l2 + m2*l1*l2*cos(theta2)).epsilon(1e-6));
        REQUIRE(M[2] == doctest::Approx(m2*l2*l2 + m2*l1*l2*cos(theta2)).epsilon(1e-6));
        REQUIRE(M[3] == doctest::Approx(m2*l2*l2).epsilon(1e-6));
    };
    auto check_dp_b = [m1, m2, l1, l2, g](float h1, float h2, float q1, float q2, float q1d, float q2d) {
        REQUIRE(h1 == doctest::Approx(
                -m2 * l1 * l2 * (2 * q1d + q2d) * q2d * sin(q2) +
                (m1+m2)*g*l1*sin(q1) + m2 * g * l2 * sin(q1 + q2)).epsilon(1e-4));
        REQUIRE(h2 == doctest::Approx(
                -m2 * l1 * l2 * q1d * q2d * sin(q2) +
                m2 * l1 * l2 * q1d * (q1d + q2d) * sin(q2) +
                +m2*g*l2*sin(q1 + q2)).epsilon(1e-4));
    };

    KinematicsData<float> kdata;
    std::vector<transform> T_global(art.get_num_joints());

    // Performance comparison.
    {
        auto t1 = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < 1000000; i++) {
            featherstone_forward_dynamics(art, glm::vec3(0, -g, 0), f_ext.data(), q.data(), qdot.data(), tau.data(), OUT q2dot_1.data());
        }
        auto t2 = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);
        MESSAGE("1000000 iters of featherstone forward dynamics: " << duration.count() << " ms");
    }

    {
        auto t1 = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < 1000000; i++) {
            forward_dynamics_using_rnea(art, glm::vec3(0, -g, 0), f_ext.data(), q.data(), qdot.data(), tau.data(), OUT
                                        q2dot_2.data());
        }
        auto t2 = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);
        MESSAGE("1000000 iters of rnea forward dynamics: " << duration.count() << " ms");
    }

    for (int i = 0; i < 1000; i++) {
        // MESSAGE("Iteration " << i);
        mass_matrix<float>(art, q.data(), OUT M.data());
        check_dp_M(M.data(), q[0], q[1]);
        rne_inverse_dynamics(art, q.data(), qdot.data(), q2dot.data(),
                             glm::vec3(0, -g, 0), f_ext.data(), OUT h.data(), OUT T_global.data());
        check_dp_b(h[0], h[1], q[0], q[1], qdot[0], qdot[1]);

        featherstone_forward_dynamics(art, glm::vec3(0, -g, 0), f_ext.data(), q.data(), qdot.data(), tau.data(), OUT q2dot_1.data());
        forward_dynamics_using_rnea(art, glm::vec3(0, -g, 0), f_ext.data(), q.data(), qdot.data(), tau.data(), OUT q2dot_2.data());

        for (int d = 0; d < dof; d++) {
            REQUIRE(q2dot_1[d] == doctest::Approx(q2dot_2[d]).epsilon(1e-6));
        }

        q2dot = q2dot_1;

        integrate_implicit_euler(art, dt, q2dot.data(), OUT q.data(), OUT qdot.data());
    }
}
