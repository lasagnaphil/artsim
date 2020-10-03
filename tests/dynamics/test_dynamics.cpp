//
// Created by Phillip Chang on 2020/09/20.
//

#include "doctest.h"

#include <artsim/artsim.h>
#include <artsim/dynamics.h>
#include <artsim/math/common.h>
#include <artsim/example_articulations.h>

#include "utils/test_utils.h"

using namespace artsim;

using real_t = float;

TEST_CASE("Helper functions for dynamics.h") {
    std::random_device random_dev;
    std::default_random_engine engine(random_dev());

    {
        tsmat6x6<real_t> A;
        tscrew<real_t> B[3];
        tscrew<real_t> C[3];

        get_random(engine, A);
        for (int i = 0; i < 3; i++) get_random<tscrew<real_t>, real_t>(engine, B[i]);

        mult_6x6_6x3(A, B, OUT C);

        Eigen::Matrix<real_t, 6, 6> Ae = to_eigen(A);
        Eigen::Matrix<real_t, 6, 3> Be = to_eigen(B);
        Eigen::Matrix<real_t, 6, 3> Cg = to_eigen(C);

        Eigen::Matrix<real_t, 6, 3> Ce = Ae * Be;
        compare_eigen(Cg, Ce)
    }
    {
        tscrew<real_t> U[3];
        tmat3x3<real_t> V;
        tscrew<real_t> UV[3];
        tsmat6x6<real_t> UVUt;

        for (int i = 0; i < 3; i++) get_random<tscrew<real_t>, real_t>(engine, U[i]);
        get_random_symmetric<real_t>(engine, V);

        mult_UVUt_6x3_3x3_3x6_sym(U, V, OUT UV, OUT UVUt);

        Eigen::Matrix<real_t, 6, 3> Ue = to_eigen(U);
        Eigen::Matrix<real_t, 3, 3> Ve = to_eigen(V);
        Eigen::Matrix<real_t, 6, 3> UVe = Ue * Ve;
        Eigen::Matrix<real_t, 6, 6> UVUte = UVe * Ue.transpose();

        Eigen::Matrix<real_t, 6, 3> UVg = to_eigen(UV);
        Eigen::Matrix<real_t, 6, 6> UVUtg = to_eigen(UVUt);

        compare_eigen(UVg, UVe)
        compare_eigen(UVUtg, UVUte)
    }
}

TEST_CASE("Double pendulum") {
    real_t m1 = 1.0f;
    real_t m2 = 1.0f;
    real_t l1 = 1.0f;
    real_t l2 = 1.0f;

    ArticulatedBody art = examples::create_double_pendulum_ball(false, m1, m2, l1, l2);
    ArticulationState state(&art);
    std::vector<real_t> q2dot_empty(state.num_vel_dofs, 0.0f);
    std::vector<real_t> q2dot_1(state.num_vel_dofs, 0.0f);
    std::vector<real_t> q2dot_2(state.num_vel_dofs, 0.0f);

    real_t g = 9.81f;
    real_t dt = 1.0f / 1000.0f;
    tvec3<real_t> gravity = {0, -g, 0};

    std::vector<real_t> M(state.num_vel_dofs*state.num_vel_dofs, 0.0f);
    std::vector<real_t> h(state.num_vel_dofs, 0.0f);

    state.q[0] = 0.25f * glm::pi<real_t>();
    state.q[1] = 0.25f * glm::pi<real_t>();
    state.qdot[0] = 0.0f;
    state.qdot[1] = 0.0f;

    auto check_dp_M = [m1, m2, l1, l2](real_t* M, real_t theta1, real_t theta2) {
        CHECK(M[0] == doctest::Approx((m1+m2)*l1*l1 + m2*l2*l2 + 2*m2*l1*l2*cos(theta2)).epsilon(1e-6));
        CHECK(M[1] == doctest::Approx(m2*l2*l2 + m2*l1*l2*cos(theta2)).epsilon(1e-6));
        CHECK(M[2] == doctest::Approx(m2*l2*l2 + m2*l1*l2*cos(theta2)).epsilon(1e-6));
        CHECK(M[3] == doctest::Approx(m2*l2*l2).epsilon(1e-6));
    };
    auto check_dp_b = [m1, m2, l1, l2, g](real_t h1, real_t h2, real_t q1, real_t q2, real_t q1d, real_t q2d) {
        CHECK(h1 == doctest::Approx(
                -m2 * l1 * l2 * (2 * q1d + q2d) * q2d * sin(q2) +
                (m1+m2)*g*l1*sin(q1) + m2 * g * l2 * sin(q1 + q2)).epsilon(1e-4));
        CHECK(h2 == doctest::Approx(
                -m2 * l1 * l2 * q1d * q2d * sin(q2) +
                m2 * l1 * l2 * q1d * (q1d + q2d) * sin(q2) +
                +m2*g*l2*sin(q1 + q2)).epsilon(1e-4));
    };

    std::vector<ttransform<real_t>> T_local(art.get_num_joints());
    std::vector<ttransform<real_t>> T_global(art.get_num_joints());

    // Performance comparison. (Featherstone currently about 2 times faster.)
    /*
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
     */

    for (int i = 0; i < 1000; i++) {
        mass_matrix<real_t>(art, state.q.data(), OUT M.data());
        check_dp_M(M.data(), state.q[0], state.q[1]);
        rne_inverse_dynamics(art, state.q.data(), state.qdot.data(), q2dot_empty.data(),
                             gravity, state.f_ext.data(), OUT h.data());
        check_dp_b(h[0], h[1], state.q[0], state.q[1], state.qdot[0], state.qdot[1]);

        featherstone_forward_dynamics(art, gravity, state.f_ext.data(), state.q.data(), state.qdot.data(), state.tau.data(), OUT q2dot_1.data());
        forward_dynamics_using_rnea(  art, gravity, state.f_ext.data(), state.q.data(), state.qdot.data(), state.tau.data(), OUT q2dot_2.data());

        // TODO: check the Featherstone method by plugging it into the Newton eq: M(q) * q2dot + C(q, qdot) = tau.

        // Compare between Featherstone and RNEA results
        for (int d = 0; d < state.num_vel_dofs; d++) {
            INFO("Iteration " << i <<", DOF " << d);
            CHECK(q2dot_1[d] == doctest::Approx(q2dot_2[d]).epsilon(1e-4));
        }

        state.q2dot = q2dot_2;

        integrate_implicit_euler(art, dt, state.q2dot.data(), OUT state.q.data(), OUT state.qdot.data());
    }
}
