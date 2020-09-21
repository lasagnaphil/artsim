//
// Created by Phillip Chang on 2020/09/20.
//

#include "doctest.h"
#include "artsim/artsim.h"
#include "artsim/dynamics.h"
#include "artsim/math/common.h"
using namespace artsim;

#include <random>
#include <chrono>
#include <Eigen/Dense>

using real_t = float;

void populate_random(std::default_random_engine& engine, real_t* buf, size_t size) {
    for (int i = 0; i < size; i++) {
        buf[i] = std::uniform_real_distribution<real_t>(-1, 1)(engine);
    }
}

template <class T, int R, int C>
Eigen::Matrix<T, R, C> to_eigen(const glm::mat<C, R, T>& M) {
    Eigen::Matrix<T, R, C> Me;
    for (int i = 0; i < R; i++) {
        for (int j = 0; j < C; j++) {
            Me(i, j) = M[j][i];
        }
    }
    return Me;
}

#define compare_glm_eigen(M, Me) \
for (int i = 0; i < R; i++) { \
    for (int j = 0; j < C; j++) { \
        REQUIRE(Me(i, j) == M[j][i].Approx()); \
    } \
}

#define compare_eigen(M1, M2) \
for (int i = 0; i < M1.rows(); i++) { \
    for (int j = 0; j < M1.cols(); j++) { \
        REQUIRE(M1(i, j) == doctest::Approx(M2(i, j))); \
    } \
}

template <class T>
Eigen::Matrix<T, 6, 1> to_eigen(const tscrew<T>& S) {
    return Eigen::Map<Eigen::Matrix<T, 6, 1>>((real_t*)&S);
}

template <class T>
Eigen::Matrix<T, 6, 3> to_eigen(tscrew<T> S[3]) {
    Eigen::Matrix<T, 6, 3> M;
    M.col(0) = to_eigen(S[0]);
    M.col(1) = to_eigen(S[1]);
    M.col(2) = to_eigen(S[2]);
    return M;
}

template <class T>
Eigen::Matrix<T, 6, 6> to_eigen(const tsmat6x6<T>& A) {
    Eigen::Matrix<real_t, 6, 6> Ae;
    Ae.block<3, 3>(0, 0) = to_eigen(A.I);
    Ae.block<3, 3>(0, 3) = to_eigen(A.C);
    Ae.block<3, 3>(3, 0) = to_eigen(A.C).transpose();
    Ae.block<3, 3>(3, 3) = to_eigen(A.M);
    return Ae;
}

TEST_CASE("Helper functions for dynamics.h") {
    std::random_device random_dev;
    std::default_random_engine engine(random_dev());

    {
        tsmat6x6<real_t> A;
        tscrew<real_t> B[3];
        tscrew<real_t> C[3];

        populate_random(engine, (real_t *) &A, sizeof(tsmat6x6<real_t>));
        populate_random(engine, (real_t *) &B, sizeof(tscrew<real_t>[3]));

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

        populate_random(engine, (real_t *) &U, sizeof(tscrew<real_t>[3]));
        populate_random(engine, (real_t *) &V, sizeof(tmat3x3<real_t>));

        mult_UVUt_6x3_3x3_3x6_sym(U, V, OUT UV, OUT UVUt);

        Eigen::Matrix<real_t, 6, 3> Ue = to_eigen(U);
        Eigen::Matrix<real_t, 3, 3> Ve = to_eigen(V);
        Eigen::Matrix<real_t, 6, 3> UVg = to_eigen(UV);
        Eigen::Matrix<real_t, 6, 6> UVUtg = to_eigen(UVUt);

        Eigen::Matrix<real_t, 6, 3> UVe = Ue * Ve;
        Eigen::Matrix<real_t, 6, 6> UVUte = UVe * Ue.transpose();
        compare_eigen(UVg, UVe);
        compare_eigen(UVUtg, UVUte);
    }
}

TEST_CASE("Double pendulum") {
    real_t m1 = 1.0f;
    real_t m2 = 1.0f;
    real_t l1 = 1.0f;
    real_t l2 = 2.0f;
    ArticulatedBody art;
    art.add_link_and_joint(
            Link::create(glm::mat3(0), m1, Shape::make_sphere(0.1f),
                         transform(glm::vec3(0.0f, -l1, 0.0f)),
                         transform(glm::vec3(0.0f, l1, 0.0f)),
                         -1, Id<Material>::null()),
            Joint::revolute_free(Ez<real_t>())
    );
    art.add_link_and_joint(
            Link::create(glm::mat3(0), m2, Shape::make_sphere(0.1f),
                         transform(glm::vec3(0.0f, -l2, 0.0f)),
                         transform(glm::vec3(0.0f, l2, 0.0f)),
                         0, Id<Material>::null()),
            Joint::revolute_free(Ez<real_t>())
    );
    art.setup();

    int dof = art.get_num_dofs();
    std::vector<real_t> q(dof, 0.0f);
    std::vector<real_t> qdot(dof, 0.0f);
    std::vector<real_t> q2dot_empty(dof, 0.0f);
    std::vector<real_t> q2dot(dof, 0.0f);
    std::vector<real_t> q2dot_1(dof, 0.0f);
    std::vector<real_t> q2dot_2(dof, 0.0f);
    std::vector<real_t> tau(dof, 0.0f);
    std::vector<tscrew<real_t>> f_ext(art.get_num_joints(), tscrew<real_t>());

    real_t g = 9.81f;
    real_t dt = 1.0f / 1000.0f;
    tvec3<real_t> gravity = {0, -g, 0};

    std::vector<real_t> M(dof*dof, 0.0f);
    std::vector<real_t> h(dof, 0.0f);

    q[0] = 0.25f * glm::pi<real_t>();
    q[1] = 0.25f * glm::pi<real_t>();
    qdot[0] = 0.0f;
    qdot[1] = 0.0f;

    auto check_dp_M = [m1, m2, l1, l2](real_t* M, real_t theta1, real_t theta2) {
        REQUIRE(M[0] == doctest::Approx((m1+m2)*l1*l1 + m2*l2*l2 + 2*m2*l1*l2*cos(theta2)).epsilon(1e-6));
        REQUIRE(M[1] == doctest::Approx(m2*l2*l2 + m2*l1*l2*cos(theta2)).epsilon(1e-6));
        REQUIRE(M[2] == doctest::Approx(m2*l2*l2 + m2*l1*l2*cos(theta2)).epsilon(1e-6));
        REQUIRE(M[3] == doctest::Approx(m2*l2*l2).epsilon(1e-6));
    };
    auto check_dp_b = [m1, m2, l1, l2, g](real_t h1, real_t h2, real_t q1, real_t q2, real_t q1d, real_t q2d) {
        REQUIRE(h1 == doctest::Approx(
                -m2 * l1 * l2 * (2 * q1d + q2d) * q2d * sin(q2) +
                (m1+m2)*g*l1*sin(q1) + m2 * g * l2 * sin(q1 + q2)).epsilon(1e-4));
        REQUIRE(h2 == doctest::Approx(
                -m2 * l1 * l2 * q1d * q2d * sin(q2) +
                m2 * l1 * l2 * q1d * (q1d + q2d) * sin(q2) +
                +m2*g*l2*sin(q1 + q2)).epsilon(1e-4));
    };

    KinematicsData<real_t> kdata;
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
        MESSAGE("Iteration " << i);
        mass_matrix<real_t>(art, q.data(), OUT M.data());
        check_dp_M(M.data(), q[0], q[1]);
        rne_inverse_dynamics(art, q.data(), qdot.data(), q2dot_empty.data(),
                             gravity, f_ext.data(), OUT h.data(), OUT T_global.data());
        check_dp_b(h[0], h[1], q[0], q[1], qdot[0], qdot[1]);

        featherstone_forward_dynamics(art, gravity, f_ext.data(), q.data(), qdot.data(), tau.data(), OUT q2dot_1.data());
        forward_dynamics_using_rnea(  art, gravity, f_ext.data(), q.data(), qdot.data(), tau.data(), OUT q2dot_2.data());

        // TODO: check the Featherstone method by plugging it into the Newton eq: M(q) * q2dot + C(q, qdot) = tau.

        // Compare between Featherstone and RNEA results
        for (int d = 0; d < dof; d++) {
            REQUIRE(q2dot_1[d] == doctest::Approx(q2dot_2[d]).epsilon(1e-4));
        }

        q2dot = q2dot_2;

        integrate_implicit_euler(art, dt, q2dot.data(), OUT q.data(), OUT qdot.data());
    }
}
