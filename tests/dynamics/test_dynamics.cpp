//
// Created by Phillip Chang on 2020/09/20.
//

#include "doctest.h"

#include <artsim/artsim.h>
#include <artsim/art_dynamics.h>
#include <artsim/math/common.h>
#include <artsim/utils/example_articulations.h>

#include "utils/test_utils.h"

#include <map>

using namespace artsim;

TEST_CASE("Double pendulum") {
    real m1 = 1.0f;
    real m2 = 1.0f;
    real l1 = 1.0f;
    real l2 = 1.0f;

    ArticulatedBodySpec spec = examples::create_double_pendulum_ball(false, m1, m2, l1, l2);
    spec.joints[0].kd = 0.0f;
    spec.joints[1].kd = 0.0f;
    ArticulatedBody art;
    art.init(spec);
    int num_vel_dofs = art.get_num_vel_dofs();
    std::vector<real> q2dot_empty(num_vel_dofs, 0.0f);
    std::vector<real> q2dot_1(num_vel_dofs, 0.0f);
    std::vector<real> q2dot_2(num_vel_dofs, 0.0f);

    real g = 9.81f;
    real dt = 1.0f / 1000.0f;
    tvec3<real> gravity = {0, -g, 0};

    dynmat<real> M1(num_vel_dofs, num_vel_dofs);
    dynmat<real> M2(num_vel_dofs, num_vel_dofs);
    M1.clear_zero();
    M2.clear_zero();
    std::vector<real> h(num_vel_dofs, 0.0f);

    art.set_joint_pos_1dof(0, 0.25 * glm::pi<real>());
    art.set_joint_pos_1dof(1, 0.25 * glm::pi<real>());
    art.set_joint_vel_1dof(0, 0.0);
    art.set_joint_vel_1dof(1, 0.0);

    auto check_dp_M = [m1, m2, l1, l2](const dynmat<real>& M, real theta1, real theta2) {
        CHECK(M(0,0) == doctest::Approx((m1+m2)*l1*l1 + m2*l2*l2 + 2*m2*l1*l2*cos(theta2)).epsilon(1e-6));
        CHECK(M(0,1) == doctest::Approx(m2*l2*l2 + m2*l1*l2*cos(theta2)).epsilon(1e-6));
        CHECK(M(1,0) == doctest::Approx(m2*l2*l2 + m2*l1*l2*cos(theta2)).epsilon(1e-6));
        CHECK(M(1,1) == doctest::Approx(m2*l2*l2).epsilon(1e-6));
    };
    auto check_dp_b = [m1, m2, l1, l2, g](real h1, real h2, real q1, real q2, real q1d, real q2d) {
        CHECK(h1 == doctest::Approx(
                -m2 * l1 * l2 * (2 * q1d + q2d) * q2d * sin(q2) +
                (m1+m2)*g*l1*sin(q1) + m2 * g * l2 * sin(q1 + q2)).epsilon(1e-4));
        CHECK(h2 == doctest::Approx(
                -m2 * l1 * l2 * q1d * q2d * sin(q2) +
                m2 * l1 * l2 * q1d * (q1d + q2d) * sin(q2) +
                +m2*g*l2*sin(q1 + q2)).epsilon(1e-4));
    };

    real* q = art.get_pos_buf();
    real* u = art.get_vel_buf();
    real* udot = art.get_acc_buf();
    tscrew<real>* f_ext = art.get_external_force_buf();
    real* tau = art.get_internal_force_buf();

    for (int i = 0; i < 1000; i++) {
        mass_matrix_using_rnea(spec, dt, q, OUT M1);
        check_dp_M(M1, q[0], q[1]);
        mass_matrix(spec, dt, q, OUT M2.to_view());
        check_dp_M(M2, q[0], q[1]);

        rne_inverse_dynamics(spec, gravity, dt, q, u, q2dot_empty.data(),
                             f_ext, OUT h.data());
        check_dp_b(h[0], h[1], q[0], q[1], u[0], u[1]);

        featherstone_forward_dynamics(spec, gravity, dt, f_ext, q, u, tau, OUT q2dot_1.data());
        forward_dynamics_using_rnea(spec, gravity, dt, f_ext, q, u, tau, OUT q2dot_2.data());

        // TODO: check the Featherstone method by plugging it into the Newton eq: M(q) * q2dot + C(q, qdot) = tau.

        // Compare between Featherstone and RNEA results
        for (int d = 0; d < num_vel_dofs; d++) {
            INFO("Iteration " << i <<", DOF " << d);
            CHECK(q2dot_1[d] == doctest::Approx(q2dot_2[d]).epsilon(1e-4));
        }

        std::copy_n(q2dot_2.data(), q2dot_2.size(), OUT udot);

        integrate_implicit_euler(spec, dt, udot, OUT q, OUT u);
    }
}

TEST_CASE("Various kinds of pendulums") {

    std::map<std::string, ArticulatedBodySpec> articulations = {
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
    for (auto& [name, spec] : articulations) {
        SUBCASE(name.c_str()) {

            std::string art_name = name;
            MESSAGE("Articulation name: " << art_name);
            ArticulatedBody art;
            art.init(spec);
            art.randomize_positions();
            int num_vel_dofs = art.get_num_vel_dofs();

            std::vector<real> q2dot_empty(num_vel_dofs, 0.0f);
            std::vector<real> q2dot_1(num_vel_dofs, 0.0f);
            std::vector<real> q2dot_2(num_vel_dofs, 0.0f);

            real g = 9.81f;
            real dt = 1.0f / 1000.0f;
            tvec3<real> gravity = {0, -g, 0};

            dynmat<real> M1(num_vel_dofs, num_vel_dofs);
            dynmat<real> M2(num_vel_dofs, num_vel_dofs);
            M1.clear_zero();
            M2.clear_zero();
            Eigen::Map<Eigen::Matrix<real, Eigen::Dynamic, Eigen::Dynamic>> M1_eigen(
                    M1.data(), num_vel_dofs, num_vel_dofs);
            Eigen::Map<Eigen::Matrix<real, Eigen::Dynamic, Eigen::Dynamic>> M2_eigen(
                    M2.data(), num_vel_dofs, num_vel_dofs);

            std::vector<real> h(num_vel_dofs, 0.0f);

            auto& spec = art.get_spec();
            real* q = art.get_pos_buf();
            real* u = art.get_vel_buf();
            real* udot = art.get_acc_buf();
            tscrew<real>* f_ext = art.get_external_force_buf();
            real* tau = art.get_internal_force_buf();

            // Performance comparison
            int num_iters = 100;
            {
                auto t1 = std::chrono::high_resolution_clock::now();
                for (int i = 0; i < num_iters; i++) {
                    featherstone_forward_dynamics(spec, glm::tvec3<real>(0, -g, 0), dt, f_ext, q, u, tau, OUT q2dot_1.data());
                }
                auto t2 = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1);
                MESSAGE(num_iters << " iters of featherstone forward dynamics: " << duration.count() << " microsecs");
            }

            for (int i = 0; i < 100; i++) {
                mass_matrix(spec, dt, q, OUT M1.to_view());

                // Only perform these tests on non-floating articulations
                if (!spec.floating) {
                    // Check if the mass matrix obtained by CRBA and RNEA are the same
                    mass_matrix_using_rnea(spec, dt, q, OUT M2);

                    SUBCASE("Mass matrix obtained by CRBA and RNEA are the same") {
                        for (int k1 = 0; k1 < num_vel_dofs; k1++) {
                            for (int k2 = 0; k2 < num_vel_dofs; k2++) {
                                CAPTURE(k1);
                                CAPTURE(k2);
                                CHECK(M1(k1, k2) == doctest::Approx(M2(k1, k2)).epsilon(1e-4));
                            }
                        }
                    }

                    // Evaluate Coriolis force
                    rne_inverse_dynamics(spec, gravity, dt, q, u, q2dot_empty.data(),
                                         f_ext, OUT h.data());

                    // Perform one step of forward dynamics using Featherstone and RNEA
                    featherstone_forward_dynamics(spec, gravity, dt, f_ext, q, u, tau, OUT q2dot_1.data());
                    forward_dynamics_using_rnea(spec, gravity, dt, f_ext, q, u, tau, OUT q2dot_2.data());

                    // Compare forward dynamics result between Featherstone and RNEA results
                    SUBCASE("Forward dynamics results obtained by Featherstone and RNEA are the same") {
                        for (int d = 0; d < num_vel_dofs; d++) {
                            CAPTURE(d);
                            CHECK(q2dot_1[d] == doctest::Approx(q2dot_2[d]).epsilon(1e-4));
                        }
                    }
                }

                // Check if the mass matrix inverse obtained by Featherstone are consistent with CRBA
                dynmat<real> Minv_using_fs(num_vel_dofs, num_vel_dofs);
                std::vector<real> tau_trial(num_vel_dofs, 0);
                std::vector<real> empty_vec(num_vel_dofs, 0);
                std::vector<tscrew<real>> empty_f_ext(num_vel_dofs, tscrew<real>(IDENTITY));

                tau_trial[0] = 1;
                featherstone_forward_dynamics(spec, tvec3<real>(0), dt,
                                              empty_f_ext.data(), q, empty_vec.data(), tau_trial.data(),
                                              OUT Minv_using_fs.data());
                for (int d = 1; d < num_vel_dofs; d++) {
                    tau_trial[d-1] = 0;
                    tau_trial[d] = 1;
                    featherstone_forward_dynamics(spec, tvec3<real>(0), dt,
                                                  empty_f_ext.data(), q, empty_vec.data(), tau_trial.data(),
                                                  OUT Minv_using_fs.data() + d * num_vel_dofs);
                }

                dynmat<real> Minv(num_vel_dofs, num_vel_dofs);
                dynmat<real> identity(num_vel_dofs, IDENTITY);
                multiply_inverse_mass_matrix(spec, dt, q, identity.to_view(), OUT Minv.to_view());

                /*
                Eigen::Map<Eigen::Matrix<real, Eigen::Dynamic, Eigen::Dynamic>> Minv_eigen_view(
                        Minv_using_fs.data(), state.num_vel_dofs, state.num_vel_dofs);
                Eigen::Map<Eigen::Matrix<real, Eigen::Dynamic, Eigen::Dynamic>> Minv_eigen_view2(
                        Minv.data(), state.num_vel_dofs, state.num_vel_dofs);

                std::cout << "FS" << std::endl;
                std::cout << Minv_eigen_view << std::endl;
                std::cout << "FS batch" << std::endl;
                std::cout << Minv_eigen_view2 << std::endl;
                 */

                SUBCASE("Mass matrix inverse obtained by Featherstone and Batch Featherstone are the same") {
                    INFO("Current ieration of loop:");
                    for (int k1 = 0; k1 < num_vel_dofs; k1++) {
                        for (int k2 = 0; k2 < num_vel_dofs; k2++) {
                            CAPTURE(k1);
                            CAPTURE(k2);
                            CHECK(Minv_using_fs(k1,k2) == doctest::Approx(Minv(k1,k2)).epsilon(1e-4));
                        }
                    }
                }

                Eigen::Matrix<real, Eigen::Dynamic, Eigen::Dynamic> M_eigen_inv = M1_eigen.inverse();

                SUBCASE("Mass matrix inverse obtained by Featherstone and CRBA are the same") {
                    for (int k1 = 0; k1 < num_vel_dofs; k1++) {
                        for (int k2 = 0; k2 < num_vel_dofs; k2++) {
                            CAPTURE(k1);
                            CAPTURE(k2);
                            CHECK(Minv_using_fs(k1, k2) == doctest::Approx(M_eigen_inv(k1, k2)).epsilon(1e-4));
                        }
                    }
                }

                // Integrate to next step using Featherstone result
                std::copy_n(q2dot_1.data(), q2dot_1.size(), udot);
                integrate_implicit_euler(spec, dt, udot, OUT q, OUT u);
            }
        }
    }
}
