//
// Created by Phillip Chang on 2020/09/20.
//

#include "doctest.h"

#include <artsim/artsim.h>
#include <artsim/dynamics.h>
#include <artsim/math/common.h>
#include <artsim/articulation_state.h>
#include <artsim/example_articulations.h>

#include "utils/test_utils.h"

#include <map>

using namespace artsim;

using real_t = double;

TEST_CASE("Double pendulum") {
    real_t m1 = 1.0f;
    real_t m2 = 1.0f;
    real_t l1 = 1.0f;
    real_t l2 = 1.0f;

    ArticulatedBody art = examples::create_double_pendulum_ball(false, m1, m2, l1, l2);
    MaterialDB material_db;
    ArticulationState<real_t> state(&art, &material_db);
    std::vector<real_t> q2dot_empty(state.num_vel_dofs, 0.0f);
    std::vector<real_t> q2dot_1(state.num_vel_dofs, 0.0f);
    std::vector<real_t> q2dot_2(state.num_vel_dofs, 0.0f);

    real_t g = 9.81f;
    real_t dt = 1.0f / 1000.0f;
    tvec3<real_t> gravity = {0, -g, 0};

    std::vector<real_t> M1(state.num_vel_dofs*state.num_vel_dofs, 0.0f);
    std::vector<real_t> M2(state.num_vel_dofs*state.num_vel_dofs, 0.0f);
    std::vector<real_t> h(state.num_vel_dofs, 0.0f);

    state.q[0] = 0.25f * glm::pi<real_t>();
    state.q[1] = 0.25f * glm::pi<real_t>();
    state.u[0] = 0.0f;
    state.u[1] = 0.0f;

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

    for (int i = 0; i < 1000; i++) {
        mass_matrix_using_rnea<real_t>(art, state.q.data(), OUT M1.data());
        check_dp_M(M1.data(), state.q[0], state.q[1]);
        mass_matrix(art, state.q.data(), OUT M2.data());
        check_dp_M(M2.data(), state.q[0], state.q[1]);

        rne_inverse_dynamics(art, state.q.data(), state.u.data(), q2dot_empty.data(),
                             gravity, state.f_ext.data(), OUT h.data());
        check_dp_b(h[0], h[1], state.q[0], state.q[1], state.u[0], state.u[1]);

        featherstone_forward_dynamics(art, gravity, state.f_ext.data(), state.q.data(), state.u.data(), state.tau.data(), OUT q2dot_1.data());
        forward_dynamics_using_rnea(art, gravity, state.f_ext.data(), state.q.data(), state.u.data(), state.tau.data(), OUT q2dot_2.data());

        // TODO: check the Featherstone method by plugging it into the Newton eq: M(q) * q2dot + C(q, qdot) = tau.

        // Compare between Featherstone and RNEA results
        for (int d = 0; d < state.num_vel_dofs; d++) {
            INFO("Iteration " << i <<", DOF " << d);
            CHECK(q2dot_1[d] == doctest::Approx(q2dot_2[d]).epsilon(1e-4));
        }

        state.udot = q2dot_2;

        integrate_implicit_euler(art, dt, state.udot.data(), OUT state.q.data(), OUT state.u.data());
    }
}

TEST_CASE("Various kinds of pendulums") {

    std::map<std::string, ArticulatedBody> articulations = {
            {"01. single link pendulum revolute", examples::create_single_pendulum_link(false)},
            {"02. single link pendulum spherical", examples::create_single_pendulum_link(true)},
            {"03. double ball pendulum revolute", examples::create_double_pendulum_ball(false)},
            {"04. double link pendulum revolute", examples::create_double_pendulum_link(false)},
            {"05. double link pendulum spherical", examples::create_double_pendulum_link(true)},
            {"06. triple link pendulum revolute", examples::create_triple_pendulum_link(false)},
            {"07. triple link pendulum spherical", examples::create_triple_pendulum_link(true)},
            {"08. furuta pendulum revolute", examples::create_furuta_pendulum(false)},
            {"09. furuta pendulum spherical", examples::create_furuta_pendulum(true)},
            {"10. 5 link tree revolute", examples::create_5_link_tree(false)},
            {"11. 5 link tree spherical", examples::create_5_link_tree(true)},
            {"12. 13 link tree revolute", examples::create_13_link_tree(false)},
            {"13. 13 link tree spherical", examples::create_13_link_tree(true)},
            {"14. floating single link", examples::create_free_link(1, false)},
            {"15. floating double link revolute", examples::create_free_link(2, false)},
            {"16. floating double link spherical", examples::create_free_link(2, true)},
    };

    MaterialDB material_db;
    for (auto& [name, art] : articulations) {
        SUBCASE(name.c_str()) {
            std::string art_name = name;
            MESSAGE("Articulation name: " << art_name);
            ArticulationState<real_t> state(&art, &material_db);
            state.randomize_positions();

            std::vector<real_t> q2dot_empty(state.num_vel_dofs, 0.0f);
            std::vector<real_t> q2dot_1(state.num_vel_dofs, 0.0f);
            std::vector<real_t> q2dot_2(state.num_vel_dofs, 0.0f);

            real_t g = 9.81f;
            real_t dt = 1.0f / 1000.0f;
            tvec3<real_t> gravity = {0, -g, 0};

            std::vector<real_t> M1(state.num_vel_dofs*state.num_vel_dofs, 0.0f);
            std::vector<real_t> M2(state.num_vel_dofs*state.num_vel_dofs, 0.0f);
            std::vector<real_t> h(state.num_vel_dofs, 0.0f);

            // Performance comparison
            int num_iters = 10000;
            {
                auto t1 = std::chrono::high_resolution_clock::now();
                for (int i = 0; i < num_iters; i++) {
                    featherstone_forward_dynamics(art, glm::tvec3<real_t>(0, -g, 0), state.f_ext.data(), state.q.data(), state.u.data(), state.tau.data(), OUT q2dot_1.data());
                }
                auto t2 = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1);
                MESSAGE(num_iters << " iters of featherstone forward dynamics: " << duration.count() << " microsecs");
            }

            {
                auto t1 = std::chrono::high_resolution_clock::now();
                for (int i = 0; i < num_iters; i++) {
                    forward_dynamics_using_rnea(art, glm::tvec3<real_t>(0, -g, 0), state.f_ext.data(), state.q.data(), state.u.data(), state.tau.data(), OUT q2dot_2.data());
                }
                auto t2 = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1);
                MESSAGE(num_iters << " iters of rnea forward dynamics: " << duration.count() << " microsecs");
            }

            for (int i = 0; i < 100; i++) {
                // Check if the mass matrix obtained by CRBA and RNEA are the same
                mass_matrix<real_t>(art, state.q.data(), OUT M1.data());
                mass_matrix_using_rnea<real_t>(art, state.q.data(), OUT M2.data());

                SUBCASE("Mass matrix obtained by CRBA and RNEA are the same") {
                    for (int k1 = 0; k1 < state.num_vel_dofs; k1++) {
                        for (int k2 = 0; k2 < state.num_vel_dofs; k2++) {
                            CHECK(M1[k1 * state.num_vel_dofs + k2] ==
                                  doctest::Approx(M2[k1 * state.num_vel_dofs + k2]).epsilon(1e-4));
                            INFO("Iteration " << i << ", DOF (" << k1 << ", " << k2 << ")");
                        }
                    }
                }

                // Print the two mass matrices
                Eigen::Map<Eigen::Matrix<real_t, Eigen::Dynamic, Eigen::Dynamic>> M1_eigen(
                        M1.data(), state.num_vel_dofs, state.num_vel_dofs);
                Eigen::Map<Eigen::Matrix<real_t, Eigen::Dynamic, Eigen::Dynamic>> M2_eigen(
                        M2.data(), state.num_vel_dofs, state.num_vel_dofs);

                // std::cout << M1_eigen << std::endl;
                // std::cout << M2_eigen << std::endl;

                // Evaluate Coriolis force
                rne_inverse_dynamics(art, state.q.data(), state.u.data(), q2dot_empty.data(),
                                     gravity, state.f_ext.data(), OUT h.data());

                // Perform one step of forward dynamics using Featherstone and RNEA
                featherstone_forward_dynamics(art, gravity, state.f_ext.data(), state.q.data(), state.u.data(), state.tau.data(), OUT q2dot_1.data());
                forward_dynamics_using_rnea(art, gravity, state.f_ext.data(), state.q.data(), state.u.data(), state.tau.data(), OUT q2dot_2.data());

                // Compare forward dynamics result between Featherstone and RNEA results
                SUBCASE("Forward dynamics results obtained by Featherstone and RNEA are the same") {
                    for (int d = 0; d < state.num_vel_dofs; d++) {
                        CHECK(q2dot_1[d] == doctest::Approx(q2dot_2[d]).epsilon(1e-4));
                    }
                }

                // Check if the mass matrix inverse obtained by Featherstone are consistent with CRBA
                std::vector<real_t> Minv_using_fs(state.num_vel_dofs*state.num_vel_dofs);
                std::vector<real_t> tau_trial(state.num_vel_dofs, 0);
                std::vector<real_t> empty_vec(state.num_vel_dofs, 0);
                std::vector<tscrew<real_t>> empty_f_ext(state.num_vel_dofs, tscrew<real_t>());

                tau_trial[0] = 1;
                featherstone_forward_dynamics(art, tvec3<real_t>(0),
                                              empty_f_ext.data(), state.q.data(), empty_vec.data(), tau_trial.data(),
                                              OUT Minv_using_fs.data());
                for (int d = 1; d < state.num_vel_dofs; d++) {
                    tau_trial[d-1] = 0;
                    tau_trial[d] = 1;
                    featherstone_forward_dynamics(art, tvec3<real_t>(0),
                                                  empty_f_ext.data(), state.q.data(), empty_vec.data(), tau_trial.data(),
                                                  OUT Minv_using_fs.data() + d * state.num_vel_dofs);
                }

                Eigen::Matrix<real_t, Eigen::Dynamic, Eigen::Dynamic> M1_eigen_inv = M1_eigen.inverse();

                SUBCASE("Mass matrix inverse obtained by Featherstone and CRBA are the same") {
                    for (int k1 = 0; k1 < state.num_vel_dofs; k1++) {
                        for (int k2 = 0; k2 < state.num_vel_dofs; k2++) {
                            INFO("Iteration " << i << ", DOF (" << k1 << ", " << k2 << ")");
                            CHECK(Minv_using_fs[k2 * state.num_vel_dofs + k1] ==
                                  doctest::Approx(M1_eigen_inv(k1,k2)).epsilon(1e-4));
                        }
                    }
                }

                // Integrate to next step using Featherstone result
                state.udot = q2dot_1;
                integrate_implicit_euler(art, dt, state.udot.data(), OUT state.q.data(), OUT state.u.data());
            }
        }
    }
}
