//
// Created by lasagnaphil on 20. 9. 10..
//

#include "doctest.h"
#include "artsim/math/common.h"
#include "artsim/math/se3.h"
#include "utils/test_utils.h"

using namespace glm;
using namespace artsim;

static std::random_device random_dev;
static std::default_random_engine engine(random_dev());
using real = float;

TEST_CASE("spmat x screw") {
    tspmat<real> A;
    tscrew<real> v;
    tscrew<real> Av;

    populate_random<decltype(A), real>(engine, &A);
    populate_random<decltype(v), real>(engine, &v);

    Av = A * v;

    Eigen::Matrix<real, 6, 6> A_e = to_eigen(tsmat6x6<real>(A));
    Eigen::Matrix<real, 6, 1> v_e = to_eigen(v);
    Eigen::Matrix<real, 6, 1> Av_e = A_e * v_e;

    Eigen::Matrix<real, 6, 1> Av_g = to_eigen(Av);

    compare_eigen(Av_g, Av_e);
}

TEST_CASE("smat6x6 x screw") {
    tsmat6x6<real> A;
    tscrew<real> B;
    tscrew<real> C;

    populate_random<decltype(A), real>(engine, &A);
    populate_random<decltype(B), real>(engine, &B);

    C = A * B;

    Eigen::Matrix<real, 6, 6> Ae = to_eigen(A);
    Eigen::Matrix<real, 6, 1> Be = to_eigen(B);
    Eigen::Matrix<real, 6, 1> Ce = Ae * Be;

    Eigen::Matrix<real, 6, 1> Cg = to_eigen(C);

    compare_eigen(Cg, Ce)
}

TEST_CASE("symmetric_cartesian_product") {
    tscrew<real> V;
    tsmat6x6<real> VVt;

    populate_random<decltype(V), real>(engine, &V);

    VVt = symmetric_cartesian_product(V);

    Eigen::Matrix<real, 6, 1> V_e = to_eigen(V);
    Eigen::Matrix<real, 6, 6> VVt_e = V_e * V_e.transpose();
    Eigen::Matrix<real, 6, 6> VVt_g = to_eigen(VVt);

    compare_eigen(VVt_g, VVt_e);
}

TEST_CASE("inv_transform") {
    ttransform<real> T_ba;
    tsmat6x6<real> G_b;
    tsmat6x6<real> G_a;

    get_random(engine, T_ba);
    get_random(engine, G_b);

    G_a = inv_transform(G_b, T_ba);

    Eigen::Matrix<real, 6, 6> Ad_T_ba_e = to_eigen_adj_matrix(T_ba);
    Eigen::Matrix<real, 6, 6> G_b_e = to_eigen(G_b);
    Eigen::Matrix<real, 6, 6> G_a_e = Ad_T_ba_e.transpose() * G_b_e * Ad_T_ba_e;
    Eigen::Matrix<real, 6, 6> G_a_g = to_eigen(G_a);

    compare_eigen(G_a_g, G_a_e);
}